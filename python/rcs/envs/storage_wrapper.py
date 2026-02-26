import datetime
import io
import operator
import os
import shutil
from concurrent.futures import ThreadPoolExecutor, wait
from queue import Queue
from typing import Any, Optional
from uuid import uuid4

import gymnasium as gym
import numpy as np
import pyarrow as pa
import pyarrow.dataset as ds
import simplejpeg
from PIL import Image


class StorageWrapper(gym.Wrapper):
    QueueSentinel = None

    def __init__(
        self,
        env: gym.Env,
        base_dir: str,
        instruction: str,
        batch_size: int = 32,
        schema: Optional[pa.Schema] = None,
        always_record: bool = False,
        basename_template: Optional[str] = None,
        max_rows_per_group: Optional[int] = None,
        max_rows_per_file: Optional[int] = None,
    ):
        """
        Asynchronously log environment transitions to a Parquet dataset on disk.

        This wrapper implements a "Crash-Safe" recording strategy:
        1. **Write-on-Receipt:** Data is written to disk in small, atomic batches immediately
           after being generated. This ensures that if the process crashes (segfault, OOM),
           previous batches are already safe on disk.
        2. **Date Partitioning:** Files are organized by date (YYYY-MM-DD) to scale to
           thousands of episodes without exhausting file system inodes.
        3. **Consolidation:** On a clean exit (`close()`), the many small batch files are
           merged into larger, optimized Parquet files.

        Observation handling:
        - Expects observations to be dictionaries.
        - RGB camera frames are JPEG-encoded.
        - Numpy arrays with ndim > 1 inside the observation dict are flattened
            in-place, and their original shapes are stored alongside as
            ``"<key>_shape"``. Nested dicts are traversed recursively.
        - Lists/tuples of arrays are not supported.
        - ``close()`` must be called to flush the final batch and run consolidation.

        Parameters
        ----------
        env : gym.Env
            The environment to wrap.
        base_dir : str
            Output directory where the Parquet dataset will be written.
        instruction : str
             A text description of the task being performed (logged in every row).
        batch_size : int, default=32
            Number of transitions to accumulate before flushing to disk.
            Smaller batches = safer against data loss but more overhead.
        schema : Optional[pa.Schema], default=None
            Optional Arrow schema. If None, inferred from the first batch.
        always_record : bool, default=False
            If True, records immediately upon reset. If False, requires start_record().
        basename_template : Optional[str], default=None
            Template for filenames. Note: A unique UUID is automatically injected
            to prevent overwrites.
        max_rows_per_group : Optional[int], default=None
            Passed to ``pyarrow.dataset.write_dataset``.
        max_rows_per_file : Optional[int], default=None
            Passed to ``pyarrow.dataset.write_dataset``.
        """
        super().__init__(env)
        self.base_dir = base_dir
        self.batch_size = batch_size
        self.schema = schema
        self.basename_template = basename_template
        self.max_rows_per_group = max_rows_per_group
        self.max_rows_per_file = max_rows_per_file
        self.buffer: list[dict[str, Any]] = []
        self.step_cnt = 0
        self._pause = not always_record
        self.always_record = always_record
        self.instruction = instruction
        self._success = False
        self._prev_action = None

        self.thread_pool = ThreadPoolExecutor()
        self.queue: Queue[pa.Table | pa.RecordBatch] = Queue(maxsize=2)
        self.uuid = uuid4()

        self._writer_future = self.thread_pool.submit(self._writer_worker)

    @staticmethod
    def consolidate(base_dir: str, schema: Optional[pa.Schema] = None):
        """
        Static method to merge small Parquet files into larger ones.
        Can be used by the class or an external CLI to clean up a dataset directory.
        """
        if not os.path.exists(base_dir):
            print(f"Directory {base_dir} does not exist.")
            return

        part_scheme = ds.partitioning(
            schema=pa.schema(fields=[pa.field("date", pa.string())]),
            flavor="filename",
        )

        print(f"Consolidating files in {base_dir}...")
        temp_dir = str(base_dir).rstrip("/") + "_temp"

        try:
            # Read existing dataset
            dataset = ds.dataset(base_dir, format="parquet", partitioning=part_scheme, schema=schema)

            try:
                if dataset.count_rows() == 0:
                    return
            except (IndexError, ValueError):
                return

            ds.write_dataset(
                data=dataset,
                base_dir=temp_dir,
                format="parquet",
                schema=schema,
                partitioning=part_scheme,
                existing_data_behavior="overwrite_or_ignore",
            )

            shutil.rmtree(base_dir)
            os.rename(temp_dir, base_dir)
            print(f"Consolidation complete for {base_dir}")

        except Exception as e:
            print(f"Consolidation failed (data remains safe in original fragments): {e}")
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir)

    def _writer_worker(self):
        """
        Background worker that writes each batch as a separate, safe file.
        Exceptions here will propagate to the future and raise RuntimeError in step().
        """
        while True:
            batch = self.queue.get()
            if batch is self.QueueSentinel:
                break

            # Generate a unique 8-char hex for this specific batch file
            unique_id = uuid4().hex[:8]

            # Handle basename template uniqueness
            if self.basename_template:
                template = self.basename_template.replace(".parquet", f"-{unique_id}.parquet")
            else:
                template = "part-{i}-" + unique_id + ".parquet"

            ds.write_dataset(
                data=batch,
                base_dir=self.base_dir,
                format="parquet",
                schema=self.schema,
                existing_data_behavior="overwrite_or_ignore",
                basename_template=template,
                max_rows_per_group=self.max_rows_per_group,
                max_rows_per_file=self.max_rows_per_file,
                partitioning=ds.partitioning(
                    schema=pa.schema(fields=[pa.field("date", pa.string())]),
                    flavor="filename",
                ),
            )

    def _flush(self):
        if self.schema is None:
            temp_batch = pa.RecordBatch.from_pylist(self.buffer)
            self.schema = temp_batch.schema

        self.buffer[-1]["success"] = self._success
        batch = pa.RecordBatch.from_pylist(self.buffer, schema=self.schema)
        self.queue.put(batch)
        self.buffer.clear()

    def _flatten_arrays(self, d: dict[str, Any]):
        # NOTE: list / tuples of arrays not supported
        updates = {}
        for k, v in d.items():
            if isinstance(v, dict):
                self._flatten_arrays(v)
            elif isinstance(v, np.ndarray) and len(v.shape) > 1:
                d[k] = v.flatten()
                updates[f"{k}_shape"] = v.shape
        d.update(updates)

    def _encode_images(self, obs: dict[str, Any]):
        # images
        _ = [
            *self.thread_pool.map(
                lambda cam: operator.setitem(
                    obs["frames"][cam]["rgb"],
                    "data",
                    simplejpeg.encode_jpeg(np.ascontiguousarray(obs["frames"][cam]["rgb"]["data"])),
                ),
                obs["frames"],
            )
        ]

        # depth
        def to_tiff(depth_data):
            img_bytes = io.BytesIO()
            Image.fromarray(
                depth_data.reshape((depth_data.shape[0], depth_data.shape[1])),
            ).save(
                img_bytes, format="TIFF"
            )  # type: ignore
            return img_bytes.getvalue()  # type: ignore

        _ = [
            *self.thread_pool.map(
                lambda cam: (
                    operator.setitem(
                        obs["frames"][cam]["depth"],
                        "data",
                        to_tiff(obs["frames"][cam]["depth"]["data"]),
                    )
                    if "depth" in obs["frames"][cam]
                    else None
                ),
                obs["frames"],
            )
        ]

    def step(self, action):
        # Check if the writer thread has died
        if self._writer_future.done():
            exc = self._writer_future.exception()
            if exc:
                msg = "Writer thread failed"
                raise RuntimeError(msg) from exc

        obs, reward, terminated, truncated, info = self.env.step(action)

        if not self._pause:
            assert isinstance(obs, dict)
            if "frames" in obs:
                self._encode_images(obs)
            self._flatten_arrays(obs)
            if info.get("success"):
                self.success()

            self.buffer.append(
                {
                    "obs": obs,
                    "reward": reward,
                    "step": self.step_cnt,
                    "uuid": self.uuid.hex,
                    "date": datetime.date.today().isoformat(),
                    "success": self._success,
                    "action": self._prev_action,
                    "instruction": self.instruction,
                    "timestamp": datetime.datetime.now().timestamp(),
                }
            )
            self._prev_action = action
            self.step_cnt += 1
            if len(self.buffer) == self.batch_size:
                self._flush()

        return obs, reward, terminated, truncated, info

    def success(self):
        self._success = True

    def stop_record(self):
        self._pause = True
        if len(self.buffer) > 0:
            self._flush()

    def start_record(self):
        self._pause = False

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        if len(self.buffer) > 0:
            self._flush()
        self._pause = not self.always_record
        self._success = False
        self._prev_action = None
        obs, info = self.env.reset()
        self.step_cnt = 0
        self.uuid = uuid4()
        return obs, info

    def close(self):
        if len(self.buffer) > 0:
            self._flush()

        self.queue.put(self.QueueSentinel)
        wait([self._writer_future])

        StorageWrapper.consolidate(self.base_dir, self.schema)
