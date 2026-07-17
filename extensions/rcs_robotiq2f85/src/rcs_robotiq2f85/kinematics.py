"""Reusable kinematics helpers."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

import rcs
from rcs._core import common

ROBOTIQ_2F85_CACHE_STEP = 0.001
_ROBOTIQ_2F85_CACHE_SIZE = 1001
_FOLLOWER_BODIES = ("left_follower", "right_follower")


@dataclass(frozen=True)
class FingerPosePair:
    """Left and right finger poses, expressed in the gripper base frame."""

    left: common.Pose
    right: common.Pose


def _pose_from_body(data: mujoco.MjData, name: str) -> common.Pose:
    body = data.body(name)
    return common.Pose(
        rotation=np.array(body.xmat, copy=True).reshape(3, 3),
        translation=np.array(body.xpos, copy=True),
    )


def _pose_from_site(data: mujoco.MjData, name: str) -> common.Pose:
    site = data.site(name)
    return common.Pose(
        rotation=np.array(site.xmat, copy=True).reshape(3, 3),
        translation=np.array(site.xpos, copy=True),
    )


@lru_cache(maxsize=None)
def _robotiq_2f85_pose_cache(model_path: str) -> np.ndarray:
    """Build the 0.001-resolution follower-pose cache once per model path."""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    actuator_id = model.actuator("fingers_actuator").id
    open_control, closed_control = model.actuator_ctrlrange[actuator_id]
    poses = np.empty((_ROBOTIQ_2F85_CACHE_SIZE, 2, 4, 4), dtype=np.float64)

    # Walk from closed to open in one simulation. Adjacent commands differ by
    # only 0.001, making cache construction much cheaper than 1001 cold starts.
    for index in range(_ROBOTIQ_2F85_CACHE_SIZE):
        normalized_state = index * ROBOTIQ_2F85_CACHE_STEP
        data.ctrl[actuator_id] = closed_control + normalized_state * (open_control - closed_control)

        for settling_step in range(2000):
            mujoco.mj_step(model, data)
            if settling_step >= 10 and np.linalg.norm(data.qvel) < 1e-5:
                break
        else:
            msg = f"Robotiq 2F-85 cache failed to converge at state {normalized_state:.3f}"
            raise RuntimeError(msg)

        for finger_index, body_name in enumerate(_FOLLOWER_BODIES):
            poses[index, finger_index] = _pose_from_body(data, body_name).pose_matrix()

    poses.setflags(write=False)
    return poses


def robotiq_2f85_finger_pose_offsets_from_sites(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    left_site: str,
    right_site: str,
    left_reference_body: str = "left_follower",
    right_reference_body: str = "right_follower",
) -> FingerPosePair:
    """Return fixed follower-to-site transforms from an existing simulation."""
    mujoco.mj_forward(model, data)
    left_reference = _pose_from_body(data, left_reference_body)
    right_reference = _pose_from_body(data, right_reference_body)
    return FingerPosePair(
        left=left_reference.inverse() * _pose_from_site(data, left_site),
        right=right_reference.inverse() * _pose_from_site(data, right_site),
    )


def robotiq_2f85_finger_poses(
    normalized_state: float,
    *,
    offsets: FingerPosePair | None = None,
    model_path: str | Path | None = None,
) -> FingerPosePair:
    """Look up Robotiq follower poses and optionally apply fixed offsets.

    ``normalized_state`` is 0 for closed and 1 for open and is rounded to the
    nearest 0.001 cache entry. The first call builds the cache using one MuJoCo
    simulation; subsequent calls only perform a lookup and pose composition.
    """
    if not np.isfinite(normalized_state) or not 0.0 <= normalized_state <= 1.0:
        msg = f"normalized_state must be between 0 and 1, got {normalized_state}"
        raise ValueError(msg)

    if model_path is None:
        model_path = Path(rcs.RCS_PREFIX) / "assets" / "grippers" / "robotiq_2f85" / "robotiq_2f85.xml"
    resolved_model_path = str(Path(model_path).resolve())
    cache_index = int(np.rint(normalized_state / ROBOTIQ_2F85_CACHE_STEP))
    cached_poses = _robotiq_2f85_pose_cache(resolved_model_path)[cache_index]
    poses = FingerPosePair(
        left=common.Pose(pose_matrix=np.array(cached_poses[0], copy=True)),
        right=common.Pose(pose_matrix=np.array(cached_poses[1], copy=True)),
    )
    if offsets is None:
        return poses
    return FingerPosePair(
        left=poses.left * offsets.left,
        right=poses.right * offsets.right,
    )
