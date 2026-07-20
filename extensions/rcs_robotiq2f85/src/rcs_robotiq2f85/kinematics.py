"""Reusable kinematics helpers."""

from __future__ import annotations

import hashlib
import logging
import os
import tempfile
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np

import rcs
from rcs._core import common

logger = logging.getLogger(__name__)

ROBOTIQ_2F85_CACHE_STEP = 0.001
_ROBOTIQ_2F85_CACHE_SIZE = 1001
_ROBOTIQ_2F85_CACHE_SHAPE = (_ROBOTIQ_2F85_CACHE_SIZE, 2, 4, 4)
_ROBOTIQ_2F85_CACHE_FORMAT_VERSION = 3
_FOLLOWER_BODIES = ("left_follower", "right_follower")


def _progress(iterable):
    try:
        from tqdm import tqdm
    except ImportError:
        return iterable
    return tqdm(iterable)


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


def _robotiq_2f85_cache_path(model_path: Path) -> Path:
    """Return a persistent cache path that is invalidated with the model or implementation."""
    fingerprint = hashlib.sha256()
    fingerprint.update(f"rcs-robotiq2f85-cache-v{_ROBOTIQ_2F85_CACHE_FORMAT_VERSION}\0".encode())
    fingerprint.update(str(model_path).encode())
    fingerprint.update(model_path.read_bytes())
    fingerprint.update(mujoco.__version__.encode())
    fingerprint.update(repr((ROBOTIQ_2F85_CACHE_STEP, _ROBOTIQ_2F85_CACHE_SHAPE, _FOLLOWER_BODIES)).encode())

    cache_root = (
        Path(os.environ["XDG_CACHE_HOME"])
        if "XDG_CACHE_HOME" in os.environ
        else Path.home() / ".cache"
    )
    return cache_root / "rcs" / "robotiq2f85" / f"{model_path.stem}-{fingerprint.hexdigest()[:16]}.npy"


def _load_robotiq_2f85_pose_cache(cache_path: Path) -> np.ndarray | None:
    try:
        poses = np.load(cache_path, allow_pickle=False)
        if poses.shape != _ROBOTIQ_2F85_CACHE_SHAPE:
            msg = f"expected shape {_ROBOTIQ_2F85_CACHE_SHAPE}, got {poses.shape}"
            raise ValueError(msg)
        if poses.dtype != np.float64:
            msg = f"expected dtype float64, got {poses.dtype}"
            raise ValueError(msg)
        if not np.all(np.isfinite(poses)):
            msg = "cache contains non-finite values"
            raise ValueError(msg)
    except FileNotFoundError:
        return None
    except (EOFError, OSError, ValueError) as exc:
        logger.warning("Ignoring invalid Robotiq 2F-85 pose cache at %s: %s", cache_path, exc)
        return None

    poses.setflags(write=False)
    return poses


def _save_robotiq_2f85_pose_cache(cache_path: Path, poses: np.ndarray) -> None:
    temporary_path: Path | None = None
    try:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            dir=cache_path.parent,
            prefix=cache_path.name,
            suffix=".tmp",
            delete=False,
        ) as file:
            temporary_path = Path(file.name)
            np.save(file, poses, allow_pickle=False)
            file.flush()
            os.fsync(file.fileno())
        temporary_path.replace(cache_path)
    except OSError as exc:
        logger.warning("Could not persist Robotiq 2F-85 pose cache at %s: %s", cache_path, exc)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                logger.debug("Could not remove temporary pose-cache file %s", temporary_path, exc_info=True)


def _build_robotiq_2f85_pose_cache(model_path: str) -> np.ndarray:
    """Build the 0.001-resolution follower-pose cache for one model."""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    actuator_id = model.actuator("fingers_actuator").id
    open_control, closed_control = model.actuator_ctrlrange[actuator_id]
    poses = np.empty(_ROBOTIQ_2F85_CACHE_SHAPE, dtype=np.float64)

    # Start open so grippers whose fingers cannot fully close do not begin in a
    # colliding state. Results retain their normalized-state cache indices.
    print(f"Building Robotiq 2F-85 pose cache for {model_path}...")
    for index in _progress(range(_ROBOTIQ_2F85_CACHE_SIZE - 1, -1, -1)):
        normalized_state = index * ROBOTIQ_2F85_CACHE_STEP
        data.ctrl[actuator_id] = closed_control + normalized_state * (open_control - closed_control)

        for settling_step in range(20000):
            mujoco.mj_step(model, data)
            if settling_step >= 10 and np.linalg.norm(data.qvel) < 1e-5:
                break
        else:
            qvel_norm = np.linalg.norm(data.qvel)
            if index == _ROBOTIQ_2F85_CACHE_SIZE - 1:
                msg = (
                    "Robotiq 2F-85 cache failed to converge at initial open state "
                    f"{normalized_state:.3f}, qvel norm {qvel_norm:.6f}"
                )
                raise RuntimeError(msg)
            previous_state = (index + 1) * ROBOTIQ_2F85_CACHE_STEP
            logger.warning(
                "Robotiq 2F-85 cache failed to converge at state %.3f (qvel norm %.6f); "
                "reusing pose from state %.3f",
                normalized_state,
                qvel_norm,
                previous_state,
            )
            poses[index] = poses[index + 1]
            continue

        for finger_index, body_name in enumerate(_FOLLOWER_BODIES):
            poses[index, finger_index] = _pose_from_body(data, body_name).pose_matrix()

    poses.setflags(write=False)
    return poses


@lru_cache(maxsize=None)
def _robotiq_2f85_pose_cache(model_path: str) -> np.ndarray:
    """Load or build the persistent follower-pose cache once per model path."""
    resolved_model_path = Path(model_path).resolve()
    cache_path = _robotiq_2f85_cache_path(resolved_model_path)
    poses = _load_robotiq_2f85_pose_cache(cache_path)
    if poses is not None:
        return poses

    poses = _build_robotiq_2f85_pose_cache(str(resolved_model_path))
    _save_robotiq_2f85_pose_cache(cache_path, poses)
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
    nearest 0.001 cache entry. The first call for a model builds the cache using
    one MuJoCo simulation and persists it below ``$XDG_CACHE_HOME/rcs`` (or
    ``~/.cache/rcs``). Subsequent processes load it from disk.
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
