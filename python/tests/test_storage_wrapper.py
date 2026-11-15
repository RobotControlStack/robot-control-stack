from typing import Any

import gymnasium as gym
import numpy as np
import pyarrow as pa
import pyarrow.dataset as ds
import pytest
from rcs.envs.storage_wrapper import StorageWrapper


class _Env(gym.Env):
    """Returns control loop states like FR3HW does and truncates the last step."""

    def __init__(self, n_steps: int):
        self.n_steps = n_steps
        self.t = 0

    def reset(self, *, seed=None, options=None):
        self.t = 0
        return {"joints": np.zeros(7)}, {}

    def step(self, action):
        self.t += 1
        n = 60 + self.t  # the number of control samples varies from step to step
        info: dict[str, Any] = {
            "robot_error": "reflex" if self.t == self.n_steps else "",
            "control_states": {
                "num_samples": n,
                "timestamp": np.arange(n, dtype=np.float64),
                "q": np.full((n, 7), float(self.t)),
                "O_T_EE": np.zeros((n, 16)),
            },
        }
        return {"joints": np.full(7, float(self.t))}, 0.0, False, self.t == self.n_steps, info


def test_storage_wrapper_stores_control_states_and_truncation(tmp_path):
    n_steps = 5
    env = StorageWrapper(_Env(n_steps), str(tmp_path), "test", batch_size=2, always_record=True)
    env.reset()
    for _ in range(n_steps):
        _, _, _, truncated, info = env.step({"joints": np.zeros(7)})
        # the caller still gets the unflattened arrays
        assert info["control_states"]["q"].ndim == 2
    assert truncated
    env.close()

    table = ds.dataset(str(tmp_path), format="parquet").to_table().sort_by("step")
    rows = table.to_pylist()
    assert len(rows) == n_steps
    assert [r["truncated"] for r in rows] == [False] * (n_steps - 1) + [True]
    assert all(r["terminated"] is False for r in rows)
    assert rows[-1]["info"]["robot_error"] == "reflex"
    for i, row in enumerate(rows):
        states = row["info"]["control_states"]
        n = 60 + i + 1
        assert states["num_samples"] == n
        q = np.asarray(states["q"]).reshape(states["q_shape"])
        assert q.shape == (n, 7)
        assert np.all(q == i + 1)
        assert np.asarray(states["O_T_EE"]).reshape(states["O_T_EE_shape"]).shape == (n, 16)


def test_storage_wrapper_annotations(tmp_path):
    env = StorageWrapper(_Env(3), str(tmp_path), "test", batch_size=2, always_record=True)
    env.reset()
    for i in range(3):
        env.annotate_step(source="policy", chunk_id=7, chunk_index=i)
        env.step({"joints": np.zeros(7)})
    env.close()
    rows = ds.dataset(str(tmp_path), format="parquet").to_table().sort_by("step").to_pylist()
    assert [r["annotations"] for r in rows] == [{"source": "policy", "chunk_id": 7, "chunk_index": i} for i in range(3)]


def test_storage_wrapper_raises_when_writer_died(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("not a directory")
    env = StorageWrapper(_Env(100), str(blocker / "data"), "test", batch_size=2, always_record=True)
    batch = pa.RecordBatch.from_pylist([{"a": 1}])
    env._put(batch)  # the writer fails on this batch because the target can not be created
    env._writer_future.exception(timeout=10)
    env.queue.put(batch)
    env.queue.put(batch)  # queue is full now, the old code would block forever here
    with pytest.raises(RuntimeError, match="Writer thread failed"):
        env._put(batch)
