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
_ROBOTIQ_2F85_CACHE_SHAPE = (_ROBOTIQ_2F85_CACHE_SIZE, 4, 4)
_ROBOTIQ_2F85_CACHE_FORMAT_VERSION = 4


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


def _robotiq_2f85_cache_path(
    model_path: Path,
    *,
    body_name: str | None = None,
    site_name: str | None = None,
) -> Path:
    """Return a persistent cache path that is invalidated with the model or implementation."""
    target_kind, target_name = _pose_cache_target(body_name=body_name, site_name=site_name)
    fingerprint = hashlib.sha256()
    fingerprint.update(f"rcs-robotiq2f85-cache-v{_ROBOTIQ_2F85_CACHE_FORMAT_VERSION}\0".encode())
    fingerprint.update(str(model_path).encode())
    fingerprint.update(model_path.read_bytes())
    fingerprint.update(mujoco.__version__.encode())
    fingerprint.update(repr((ROBOTIQ_2F85_CACHE_STEP, _ROBOTIQ_2F85_CACHE_SHAPE, target_kind, target_name)).encode())

    cache_root = Path(os.environ["XDG_CACHE_HOME"]) if "XDG_CACHE_HOME" in os.environ else Path.home() / ".cache"
    return cache_root / "rcs" / "robotiq2f85" / f"{model_path.stem}-{fingerprint.hexdigest()[:16]}.npy"


def _pose_cache_target(*, body_name: str | None, site_name: str | None) -> tuple[str, str]:
    if (body_name is None) == (site_name is None):
        msg = "exactly one of body_name or site_name must be provided"
        raise ValueError(msg)
    if body_name is not None:
        return "body", body_name
    assert site_name is not None
    return "site", site_name


def _load_robotiq_2f85_pose_cache(cache_path: Path) -> np.ndarray | None:
    try:
        loaded_poses = np.load(cache_path, allow_pickle=False)
        if loaded_poses.dtype != np.float64:
            msg = f"expected dtype float64, got {loaded_poses.dtype}"
            raise ValueError(msg)
        # Keep every pose resident in memory. In particular, do not retain a
        # memory-mapped or otherwise lazy array returned by the loader.
        poses = np.array(loaded_poses, order="C", copy=True)
        if poses.shape != _ROBOTIQ_2F85_CACHE_SHAPE:
            msg = f"expected shape {_ROBOTIQ_2F85_CACHE_SHAPE}, got {poses.shape}"
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


def _build_robotiq_2f85_pose_cache(
    model_path: str,
    *,
    body_name: str | None = None,
    site_name: str | None = None,
) -> np.ndarray:
    """Build a 0.001-resolution body- or site-pose cache for one model."""
    target_kind, target_name = _pose_cache_target(body_name=body_name, site_name=site_name)
    pose_from_target = _pose_from_body if target_kind == "body" else _pose_from_site
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

        poses[index] = pose_from_target(data, target_name).pose_matrix()

    poses.setflags(write=False)
    return poses


@lru_cache(maxsize=None)
def _robotiq_2f85_pose_cache(
    model_path: str,
    *,
    body_name: str | None = None,
    site_name: str | None = None,
) -> np.ndarray:
    """Load or build one persistent body- or site-pose cache."""
    _pose_cache_target(body_name=body_name, site_name=site_name)
    resolved_model_path = Path(model_path).resolve()
    cache_path = _robotiq_2f85_cache_path(resolved_model_path, body_name=body_name, site_name=site_name)
    poses = _load_robotiq_2f85_pose_cache(cache_path)
    if poses is not None:
        return poses

    poses = _build_robotiq_2f85_pose_cache(str(resolved_model_path), body_name=body_name, site_name=site_name)
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
    body_name: tuple[str, str] | None = None,
    site_name: tuple[str, str] | None = None,
    offsets: FingerPosePair | None = None,
    model_path: str | Path | None = None,
) -> FingerPosePair:
    """Look up a pair of Robotiq body or site poses and optionally apply offsets.

    ``normalized_state`` is 0 for closed and 1 for open and is rounded to the
    nearest 0.001 cache entry. Exactly one pair of ``body_name`` or
    ``site_name`` values must be provided, ordered left then right. The first
    call for each target builds its cache and persists it below
    ``$XDG_CACHE_HOME/rcs`` (or ``~/.cache/rcs``). Subsequent processes load it
    from disk.
    """
    if not np.isfinite(normalized_state) or not 0.0 <= normalized_state <= 1.0:
        msg = f"normalized_state must be between 0 and 1, got {normalized_state}"
        raise ValueError(msg)
    if (body_name is None) == (site_name is None):
        msg = "exactly one of body_name or site_name must be provided"
        raise ValueError(msg)

    target_kind = "body" if body_name is not None else "site"
    target_names = body_name if body_name is not None else site_name
    if (
        not isinstance(target_names, tuple)
        or len(target_names) != 2
        or not all(isinstance(name, str) for name in target_names)
    ):
        msg = f"{target_kind}_name must be a pair of names ordered left then right"
        raise ValueError(msg)

    if model_path is None:
        model_path = Path(rcs.RCS_PREFIX) / "assets" / "grippers" / "robotiq_2f85" / "robotiq_2f85.xml"
    resolved_model_path = str(Path(model_path).resolve())
    cache_index = int(np.rint(normalized_state / ROBOTIQ_2F85_CACHE_STEP))
    if body_name is not None:
        cached_poses = tuple(
            _robotiq_2f85_pose_cache(resolved_model_path, body_name=name)[cache_index] for name in target_names
        )
    else:
        cached_poses = tuple(
            _robotiq_2f85_pose_cache(resolved_model_path, site_name=name)[cache_index] for name in target_names
        )
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
