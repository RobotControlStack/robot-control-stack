#!/usr/bin/env python3
"""Average the first observed Franka joint pose from every LMDB episode."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import lmdb
import msgpack
import numpy as np


LMDB_FORMAT = "rcs.lmdb_joint.v1"
ARM_DOF = 7
DEFAULT_ROBOT_KEY = "right"
DEFAULT_LMDB_PATH = Path(__file__).resolve().parents[3] / "datasets/data_lmdb/rumi_pnp_new"


def _decode_lmdb_object(value: dict[str, Any]) -> Any:
    """Decode NumPy arrays written by RCS's LMDB converter."""
    if not (value.get("__ndarray__") or value.get("__nd__")):
        return value

    return np.frombuffer(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"]).copy()


def _unpack(raw: bytes | None, key: bytes) -> dict[str, Any]:
    if raw is None:
        message = f"LMDB key is missing: {key!r}"
        raise KeyError(message)
    return msgpack.unpackb(raw, raw=False, object_hook=_decode_lmdb_object)


def average_episode_start_joints(
    lmdb_path: Path,
    robot_key: str = DEFAULT_ROBOT_KEY,
) -> np.ndarray:
    lmdb_path = lmdb_path.expanduser().resolve()
    if not lmdb_path.exists():
        message = f"LMDB dataset does not exist: {lmdb_path}"
        raise FileNotFoundError(message)

    env = lmdb.open(
        str(lmdb_path),
        subdir=lmdb_path.is_dir(),
        readonly=True,
        lock=False,
        readahead=False,
        meminit=False,
        max_readers=1,
    )

    try:
        with env.begin(buffers=False) as transaction:
            meta = _unpack(transaction.get(b"meta"), b"meta")
            if meta.get("format") != LMDB_FORMAT:
                message = f"Unsupported LMDB format {meta.get('format')!r}; " f"expected {LMDB_FORMAT!r}"
                raise ValueError(message)

            total_episodes = int(meta["total_episodes"])
            episode_starts = [int(index) for index in meta["episode_from_indices"]]
            episode_stops = [int(index) for index in meta["episode_to_indices"]]
            if total_episodes == 0:
                message = "No episodes were found."
                raise ValueError(message)
            if len(episode_starts) != total_episodes or len(episode_stops) != total_episodes:
                message = (
                    "LMDB episode boundaries do not match total_episodes: "
                    f"starts={len(episode_starts)}, stops={len(episode_stops)}, "
                    f"total={total_episodes}"
                )
                raise ValueError(message)

            state_feature = meta.get("features", {}).get("observation.state", {})
            state_names = state_feature.get("names")
            if state_names is None:
                message = "observation.state does not define component names"
                raise ValueError(message)

            expected_joint_names = [f"{robot_key}_joint_{index}" for index in range(ARM_DOF)]
            missing_names = [name for name in expected_joint_names if name not in state_names]
            if missing_names:
                message = (
                    f"observation.state is missing joint fields {missing_names}; "
                    f"available fields are {list(state_names)}"
                )
                raise ValueError(message)
            joint_indices = [state_names.index(name) for name in expected_joint_names]

            start_joints: list[np.ndarray] = []
            for episode_index, (global_start, global_stop) in enumerate(
                zip(episode_starts, episode_stops, strict=True)
            ):
                if global_start >= global_stop:
                    message = f"Episode {episode_index} is empty: [{global_start}, {global_stop})"
                    raise ValueError(message)

                key = f"frame/{global_start:09d}".encode("ascii")
                record = _unpack(transaction.get(key), key)
                record_episode = int(record["episode_index"])
                if record_episode != episode_index:
                    message = f"First frame for episode {episode_index} reports episode " f"{record_episode}"
                    raise ValueError(message)

                state = np.asarray(record["observation.state"], dtype=np.float64).reshape(-1)
                if state.size != len(state_names):
                    message = (
                        f"Episode {episode_index} has observation.state size {state.size}, "
                        f"but metadata defines {len(state_names)} fields"
                    )
                    raise ValueError(message)

                joints = state[joint_indices]
                if joints.shape != (ARM_DOF,):
                    message = f"Episode {episode_index} has unexpected joint shape {joints.shape}"
                    raise ValueError(message)
                if not np.isfinite(joints).all():
                    message = f"Episode {episode_index} contains non-finite joint values"
                    raise ValueError(message)

                start_joints.append(joints)
                frame_index = int(record.get("frame_index", 0))
                print(
                    f"Episode {episode_index}, first frame {frame_index} "
                    f"(global {global_start}): "
                    f"{np.array2string(joints, precision=8, separator=', ')}"
                )
    finally:
        env.close()

    stacked_joints = np.stack(start_joints)
    average_joints = np.mean(stacked_joints, axis=0)
    standard_deviation = np.std(stacked_joints, axis=0)

    print(f"\nEpisodes: {len(stacked_joints)}")
    print("Average q_home [rad]: " f"{np.array2string(average_joints, precision=8, separator=', ')}")
    print("Start-pose standard deviation [rad]: " f"{np.array2string(standard_deviation, precision=8, separator=', ')}")

    return average_joints


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "lmdb_path",
        type=Path,
        nargs="?",
        default=DEFAULT_LMDB_PATH,
        help=f"RCS LMDB dataset (default: {DEFAULT_LMDB_PATH})",
    )
    parser.add_argument(
        "--robot-key",
        default=DEFAULT_ROBOT_KEY,
        help=f"Robot prefix in observation.state (default: {DEFAULT_ROBOT_KEY})",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    average_episode_start_joints(arguments.lmdb_path, arguments.robot_key)
