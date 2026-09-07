#!/usr/bin/env python3
"""Replay one joint trajectory from an RCS LMDB dataset on a Franka.

The recorded ``observation.state`` is sampled before the corresponding
``action`` is applied. This script preserves that alignment: the live robot
state is captured immediately before each action and compared with the LMDB
observation at the same frame.

Example:

    python examples/inference/franka_rumi_lmdb_replay.py --episode 0

Use ``--inspect-only`` to validate the dataset and selected frame range without
connecting to or moving the robot.
"""

from __future__ import annotations

import argparse
import json
import logging
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from time import perf_counter
from typing import Any

import lmdb
import msgpack
import numpy as np

LOGGER = logging.getLogger(__name__)

LMDB_FORMAT = "rcs.lmdb_joint.v1"
ROBOT_KEY = "right"
ARM_DOF = 7
STATE_DIM = ARM_DOF + 1
DEFAULT_ROBOT_IP = "192.168.102.1"
DEFAULT_DATASET = Path(__file__).resolve().parents[3] / "datasets/data_lmdb/rumi_debug_3"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "replay_results"
DEFAULT_MAX_JOINT_STEP_DEG = 5
ACTION_SPACES = ("joints", "tquat", "xyzrpy", "delta_tquat", "delta_xyzrpy")


def _decode_lmdb_object(value: dict[str, Any]) -> Any:
    """Decode numpy arrays written by RCS's LMDBJointDatasetConverter."""
    if not (value.get("__ndarray__") or value.get("__nd__")):
        return value
    return np.frombuffer(value["data"], dtype=np.dtype(value["dtype"])).reshape(value["shape"]).copy()


def _unpack(raw: bytes | None, key: bytes) -> dict[str, Any]:
    if raw is None:
        msg = f"LMDB key is missing: {key!r}"
        raise KeyError(msg)
    return msgpack.unpackb(raw, raw=False, object_hook=_decode_lmdb_object)


@dataclass(frozen=True)
class Episode:
    index: int
    task: str
    fps: float
    global_indices: np.ndarray
    frame_indices: np.ndarray
    timestamps: np.ndarray
    observations: np.ndarray
    actions: np.ndarray
    action_space: str
    total_episodes: int
    full_episode_length: int


def load_episode(
    dataset_path: Path,
    episode_index: int,
    start: int,
    stop: int | None,
    action_space: str = "joints",
) -> Episode:
    """Load and validate a frame range from one LMDB episode."""
    dataset_path = dataset_path.expanduser().resolve()
    if not dataset_path.is_file():
        msg = f"LMDB dataset does not exist or is not a single file: {dataset_path}"
        raise FileNotFoundError(msg)

    env = lmdb.open(
        str(dataset_path),
        subdir=False,
        readonly=True,
        lock=False,
        readahead=False,
        meminit=False,
        max_readers=1,
    )
    try:
        with env.begin(buffers=False) as txn:
            meta = _unpack(txn.get(b"meta"), b"meta")
            if meta.get("format") != LMDB_FORMAT:
                msg = f"Unsupported LMDB format {meta.get('format')!r}; expected {LMDB_FORMAT!r}"
                raise ValueError(msg)

            total_episodes = int(meta["total_episodes"])
            if not 0 <= episode_index < total_episodes:
                msg = f"episode {episode_index} is outside the valid range [0, {total_episodes})"
                raise IndexError(msg)

            global_episode_start = int(meta["episode_from_indices"][episode_index])
            global_episode_stop = int(meta["episode_to_indices"][episode_index])
            episode_length = global_episode_stop - global_episode_start
            selected_stop = episode_length if stop is None else stop
            if not 0 <= start < selected_stop <= episode_length:
                msg = (
                    f"invalid frame range [{start}, {selected_stop}) for episode {episode_index} "
                    f"with {episode_length} frames"
                )
                raise ValueError(msg)

            features = meta.get("features", {})
            state_shape = tuple(features.get("observation.state", {}).get("shape", ()))
            action_shape = tuple(features.get("action", {}).get("shape", ()))
            expected_action_dim = {
                "joints": 8,
                "tquat": 8,
                "delta_tquat": 8,
                "xyzrpy": 7,
                "delta_xyzrpy": 7,
            }[action_space]
            if state_shape != (STATE_DIM,) or action_shape != (expected_action_dim,):
                msg = (
                    "This replay expects 7 joints plus 1 gripper observation and "
                    f"an {action_space} action with {expected_action_dim} values; "
                    f"got observation.state={state_shape}, action={action_shape}"
                )
                raise ValueError(msg)

            state_names = features.get("observation.state", {}).get("names")
            action_names = features.get("action", {}).get("names")
            expected_names = [f"{ROBOT_KEY}_joint_{i}" for i in range(ARM_DOF)] + [f"{ROBOT_KEY}_gripper"]
            if state_names is not None and list(state_names) != expected_names:
                msg = f"Unexpected observation.state layout: {state_names!r}"
                raise ValueError(msg)
            if action_space == "joints" and action_names is not None and list(action_names) != expected_names:
                msg = f"Unexpected joint action layout: {action_names!r}"
                raise ValueError(msg)
            expected_action_components = {
                "tquat": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                "delta_tquat": ["delta_x", "delta_y", "delta_z", "delta_qx", "delta_qy", "delta_qz", "delta_qw"],
                "xyzrpy": ["x", "y", "z", "roll", "pitch", "yaw"],
                "delta_xyzrpy": ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"],
            }
            if action_space != "joints" and action_names is not None:
                expected_action_names = [
                    f"{ROBOT_KEY}_{name}" for name in expected_action_components[action_space]
                ] + [f"{ROBOT_KEY}_gripper"]
                if list(action_names) != expected_action_names:
                    msg = (
                        f"Action-space mismatch: --action-space {action_space!r} expects "
                        f"{expected_action_names!r}, got {list(action_names)!r}"
                    )
                    raise ValueError(msg)

            global_indices = np.arange(
                global_episode_start + start,
                global_episode_start + selected_stop,
                dtype=np.int64,
            )
            records: list[dict[str, Any]] = []
            for global_index in global_indices:
                key = f"frame/{int(global_index):09d}".encode("ascii")
                record = _unpack(txn.get(key), key)
                if int(record["episode_index"]) != episode_index:
                    msg = (
                        f"LMDB frame {global_index} belongs to episode {record['episode_index']}, "
                        f"not episode {episode_index}"
                    )
                    raise ValueError(msg)
                records.append(record)
    finally:
        env.close()

    observations = np.stack([record["observation.state"] for record in records]).astype(np.float64)
    actions = np.stack([record["action"] for record in records]).astype(np.float64)
    if not np.isfinite(observations).all() or not np.isfinite(actions).all():
        msg = "Selected trajectory contains non-finite observation or action values"
        raise ValueError(msg)

    task = str(meta.get("episode_tasks", [""] * total_episodes)[episode_index])
    return Episode(
        index=episode_index,
        task=task,
        fps=float(meta["fps"]),
        global_indices=global_indices,
        frame_indices=np.asarray([record["frame_index"] for record in records], dtype=np.int64),
        timestamps=np.asarray([record["timestamp"] for record in records], dtype=np.float64),
        observations=observations,
        actions=actions,
        action_space=action_space,
        total_episodes=total_episodes,
        full_episode_length=episode_length,
    )


def build_hardware_env(
    initial_joints: np.ndarray,
    robot_ip: str,
    gripper_type: str,
    max_joint_step_deg: float,
    speed_factor: float,
    action_space: str = "joints",
    integrate_deltas_from_command: bool = False,
):
    """Build the same single-arm hardware stack used by franka_rumi.py."""
    import rcs
    from rcs._core.common import GripperType
    from rcs.envs.base import ControlMode, RelativeTo
    from rcs_fr3.configs import SingleArmFR3MultiHardwareEnv

    selected_gripper = GripperType("Robotiq2F85") if gripper_type == "robotiq" else GripperType.FrankaHand
    creator = SingleArmFR3MultiHardwareEnv()
    creator.ip = robot_ip
    cfg = creator.config(grippertype=selected_gripper, robot_ip=robot_ip)
    if action_space == "joints":
        cfg.control_mode = ControlMode.JOINTS
        cfg.relative_to = RelativeTo.NONE
        cfg.max_relative_movement = None if max_joint_step_deg <= 0 else float(np.deg2rad(max_joint_step_deg))
    elif action_space in {"xyzrpy", "delta_xyzrpy"}:
        cfg.control_mode = ControlMode.CARTESIAN_TRPY
        cfg.relative_to = (
            RelativeTo.NONE
            if integrate_deltas_from_command and action_space == "delta_xyzrpy"
            else RelativeTo.LAST_STEP
        )
        cfg.max_relative_movement = (0.5, np.deg2rad(90))
    else:
        cfg.control_mode = ControlMode.CARTESIAN_TQuat
        cfg.relative_to = (
            RelativeTo.NONE
            if integrate_deltas_from_command and action_space == "delta_tquat"
            else RelativeTo.LAST_STEP
        )
        cfg.max_relative_movement = (0.5, np.deg2rad(90))
    if integrate_deltas_from_command and action_space in {"delta_tquat", "delta_xyzrpy"}:
        # RelativeTo.NONE installs LimitedAbsoluteAction when this is set.
        # That wrapper clips absolute targets relative to the measured pose,
        # which defeats integration from the last commanded pose.
        cfg.max_relative_movement = None
    cfg.camera_cfgs = None
    cfg.robot_to_shared_base_frame = {
        ROBOT_KEY: rcs.common.Pose(
            translation=np.zeros(3, dtype=np.float64),
            rpy_vector=np.zeros(3, dtype=np.float64),
        )
    }
    cfg.wrapper_cfg.binary_gripper = False

    robot_cfg = cfg.robot_cfgs[ROBOT_KEY]
    robot_cfg.ignore_realtime = True
    robot_cfg.speed_factor = speed_factor
    robot_cfg.q_home = np.asarray(initial_joints, dtype=np.float64)

    # Match the gains and torque limits used for the RUMI setup.
    robot_cfg.joint_controller_Kp = 20 * np.asarray([24, 24, 24, 24, 10, 6, 3], dtype=np.float64)
    robot_cfg.joint_controller_Kd = 2 * np.sqrt(robot_cfg.joint_controller_Kp)
    robot_cfg.joint_controller_torque_limits = np.asarray([12.0, 12.0, 12.0, 10.0, 5.0, 4.0, 3.0])
    robot_cfg.osc_Kp_p = 2 *  np.asarray([150, 150, 150])
    robot_cfg.osc_Kp_r = 1.5 *  np.asarray([250, 250, 250])
    robot_cfg.osc_torque_limits = np.asarray([12.0, 12.0, 12.0, 10.0, 5.0, 4.0, 3.0])
    return creator.create_env(cfg)


def _state_from_observation(observation: dict[str, Any]) -> np.ndarray:
    try:
        robot_observation = observation[ROBOT_KEY]
        joints = np.asarray(robot_observation["joints"], dtype=np.float64)
        gripper = np.asarray(robot_observation["gripper"], dtype=np.float64).reshape(-1)
    except (KeyError, TypeError) as exc:
        msg = f"Unexpected RCS observation structure: {observation.keys()}"
        raise ValueError(msg) from exc
    if joints.shape != (ARM_DOF,) or gripper.shape != (1,):
        msg = f"Unexpected live state shapes: joints={joints.shape}, gripper={gripper.shape}"
        raise ValueError(msg)
    return np.concatenate([joints, gripper])


def _quaternion_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Multiply xyzw quaternions."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=np.float64,
    )


def _quaternion_to_rpy(quaternion: np.ndarray) -> np.ndarray:
    """Convert a relative xyzw quaternion to a shortest local RPY increment."""
    quaternion = np.asarray(quaternion, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion = -quaternion
    vector = quaternion[:3]
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1e-12:
        return 2.0 * vector
    angle = 2.0 * np.arctan2(vector_norm, quaternion[3])
    return vector * (angle / vector_norm)


def _cumulative_cartesian_motion(actions: np.ndarray, action_space: str) -> dict[str, Any]:
    """Calculate signed and path-length Cartesian motion for absolute actions."""
    arm_actions = np.asarray(actions[:, :-1], dtype=np.float64)
    if len(arm_actions) < 2:
        zero = np.zeros(3, dtype=np.float64)
        return {
            "translation": zero.tolist(),
            "translation_norm": 0.0,
            "translation_path_length": 0.0,
            "rotation_rpy": zero.tolist(),
            "rotation_norm_rad": 0.0,
            "rotation_norm_deg": 0.0,
            "rotation_path_length_rad": 0.0,
            "rotation_path_length_deg": 0.0,
        }

    if action_space == "xyzrpy":
        positions = arm_actions[:, :3]
        rpy = np.unwrap(arm_actions[:, 3:6], axis=0)
        translation_steps = np.diff(positions, axis=0)
        rotation_steps = np.diff(rpy, axis=0)
    elif action_space == "tquat":
        positions = arm_actions[:, :3]
        quaternions = arm_actions[:, 3:7].copy()
        norms = np.linalg.norm(quaternions, axis=1)
        if np.any(norms == 0):
            raise ValueError("tquat actions contain a zero-length quaternion")
        quaternions /= norms[:, None]
        # q and -q represent the same orientation; choose one continuous branch.
        for index in range(1, len(quaternions)):
            if np.dot(quaternions[index - 1], quaternions[index]) < 0:
                quaternions[index] *= -1.0
        translation_steps = np.diff(positions, axis=0)
        rotation_steps = np.asarray(
            [
                _quaternion_to_rpy(
                    _quaternion_multiply(quaternions[index + 1], quaternions[index] * np.array([-1, -1, -1, 1]))
                )
                for index in range(len(quaternions) - 1)
            ]
        )
    else:
        raise ValueError(f"Cumulative Cartesian motion is not defined for {action_space!r} actions")

    translation = np.sum(translation_steps, axis=0)
    rotation_rpy = np.sum(rotation_steps, axis=0)
    return {
        "translation": translation.tolist(),
        "translation_norm": float(np.linalg.norm(translation)),
        "translation_path_length": float(np.linalg.norm(translation_steps, axis=1).sum()),
        "rotation_rpy": rotation_rpy.tolist(),
        "rotation_norm_rad": float(np.linalg.norm(rotation_rpy)),
        "rotation_norm_deg": float(np.rad2deg(np.linalg.norm(rotation_rpy))),
        "rotation_path_length_rad": float(np.linalg.norm(rotation_steps, axis=1).sum()),
        "rotation_path_length_deg": float(np.rad2deg(np.linalg.norm(rotation_steps, axis=1).sum())),
    }


def _action_for_environment(
    action_vector: np.ndarray, action_space: str, observation: dict[str, Any]
) -> dict[str, dict[str, np.ndarray]]:
    """Convert the selected LMDB action representation to the env action format."""
    gripper = action_vector[-1:].astype(np.float32)
    arm_action = action_vector[:-1]
    if action_space == "joints":
        if arm_action.shape != (ARM_DOF,):
            raise ValueError(f"Expected 7 joint action values, got {arm_action.shape}")
        arm_key = "joints"
        converted = arm_action.astype(np.float32)
    else:
        import rcs

        current_tquat = np.asarray(observation[ROBOT_KEY]["tquat"], dtype=np.float64)
        current_pose = rcs.common.Pose(translation=current_tquat[:3], quaternion=current_tquat[3:])
        if action_space in {"tquat", "delta_tquat"}:
            if arm_action.shape != (7,):
                raise ValueError(f"Expected 7 {action_space} action values, got {arm_action.shape}")
            if action_space == "delta_tquat":
                delta_pose = rcs.common.Pose(translation=arm_action[:3], quaternion=arm_action[3:])
            else:
                target_pose = rcs.common.Pose(translation=arm_action[:3], quaternion=arm_action[3:])
                delta_pose = target_pose * current_pose.inverse()
        else:
            if arm_action.shape != (6,):
                raise ValueError(f"Expected 6 {action_space} action values, got {arm_action.shape}")
            if action_space == "delta_xyzrpy":
                arm_key = "xyzrpy"
                converted = arm_action.astype(np.float32)
                return {ROBOT_KEY: {arm_key: converted, "gripper": gripper}}
            else:
                target_pose = rcs.common.Pose(translation=arm_action[:3], rpy_vector=arm_action[3:])
                delta_pose = target_pose * current_pose.inverse()
                arm_key = "xyzrpy"
                converted = np.concatenate(
                    [
                        target_pose.translation() - current_pose.translation(),
                        delta_pose.rotation_rpy().as_vector(),
                    ]
                ).astype(np.float32)
                return {ROBOT_KEY: {arm_key: converted, "gripper": gripper}}
        arm_key = "tquat"
        converted = np.concatenate([delta_pose.translation(), delta_pose.rotation_q()]).astype(np.float32)

    return {ROBOT_KEY: {arm_key: converted, "gripper": gripper}}


def _cartesian_command_delta(
    action: dict[str, dict[str, np.ndarray]], action_space: str
) -> np.ndarray:
    """Return the Cartesian command sent to the controller as xyzrpy delta."""
    robot_action = action[ROBOT_KEY]
    if action_space in {"xyzrpy", "delta_xyzrpy"}:
        return np.asarray(robot_action["xyzrpy"], dtype=np.float64)
    delta_tquat = np.asarray(robot_action["tquat"], dtype=np.float64)
    return np.concatenate(
        [delta_tquat[:3], _quaternion_to_rpy(delta_tquat[3:])]
    )


def _observed_cartesian_delta(
    previous_observation: dict[str, Any], observation: dict[str, Any]
) -> np.ndarray:
    """Return the measured Cartesian increment as xyzrpy delta."""
    previous_tquat = np.asarray(previous_observation[ROBOT_KEY]["tquat"], dtype=np.float64)
    current_tquat = np.asarray(observation[ROBOT_KEY]["tquat"], dtype=np.float64)
    previous_quaternion = previous_tquat[3:] / np.linalg.norm(previous_tquat[3:])
    current_quaternion = current_tquat[3:] / np.linalg.norm(current_tquat[3:])
    relative_quaternion = _quaternion_multiply(
        current_quaternion,
        previous_quaternion * np.array([-1.0, -1.0, -1.0, 1.0]),
    )
    return np.concatenate(
        [
            current_tquat[:3] - previous_tquat[:3],
            _quaternion_to_rpy(relative_quaternion),
        ]
    )


def _integrated_delta_action(
    action_vector: np.ndarray,
    action_space: str,
    commanded_pose: Any,
) -> tuple[dict[str, dict[str, np.ndarray]], Any]:
    """Integrate a Cartesian delta from the previous commanded pose."""
    import rcs

    gripper = action_vector[-1:].astype(np.float32)
    arm_action = action_vector[:-1]
    if action_space == "delta_tquat":
        if arm_action.shape != (7,):
            raise ValueError(f"Expected 7 delta_tquat action values, got {arm_action.shape}")
        delta_pose = rcs.common.Pose(
            translation=arm_action[:3].reshape(3, 1),
            quaternion=arm_action[3:].reshape(4, 1),
        )
        arm_key = "tquat"
    elif action_space == "delta_xyzrpy":
        if arm_action.shape != (6,):
            raise ValueError(f"Expected 6 delta_xyzrpy action values, got {arm_action.shape}")
        # delta_xyzrpy translation is an XYZ position difference in the
        # controller/world frame. It must be added directly; composing a full
        # rigid transform would rotate the existing commanded translation when
        # applying the RPY increment.
        delta_rotation = rcs.common.Pose(rpy_vector=arm_action[3:].reshape(3, 1), translation=np.zeros(3, dtype=np.float64))
        arm_key = "xyzrpy"
    else:
        raise ValueError(f"Integrated command replay requires a delta action, got {action_space!r}")

    if action_space == "delta_xyzrpy":
        next_rotation = delta_rotation * rcs.common.Pose(
            quaternion=commanded_pose.rotation_q().reshape(4, 1)
        )
        next_commanded_pose = rcs.common.Pose(
            translation=(np.asarray(commanded_pose.translation()).reshape(-1) + arm_action[:3]).reshape(3, 1),
            quaternion=next_rotation.rotation_q().reshape(4, 1),
        )
    else:
        next_commanded_pose = delta_pose * commanded_pose
    target = (
        np.concatenate(
            [
                np.asarray(next_commanded_pose.translation()).reshape(-1),
                np.asarray(next_commanded_pose.rotation_q()).reshape(-1),
            ]
        )
        if arm_key == "tquat"
        else np.asarray(next_commanded_pose.xyzrpy()).reshape(-1)
    )
    return {
        ROBOT_KEY: {arm_key: np.asarray(target, dtype=np.float32), "gripper": gripper}
    }, next_commanded_pose


@dataclass(frozen=True)
class ReplayTrace:
    actual_states: np.ndarray
    wall_times: np.ndarray
    commanded_cartesian_deltas: np.ndarray
    observed_cartesian_deltas: np.ndarray
    completed: int
    interrupted: bool


def replay(
    env,
    episode: Episode,
    fps: float,
    integrate_deltas_from_command: bool = False,
) -> ReplayTrace:
    """Reset to the selected start state and replay actions at a fixed rate."""
    from rcs.utils import SimpleFrameRate

    observation, _ = env.reset()
    rate = SimpleFrameRate(fps, "LMDB trajectory replay")
    rate()  # Prime the clock so the first command is also held for one period.
    start_time = perf_counter()
    actual_states: list[np.ndarray] = []
    wall_times: list[float] = []
    commanded_cartesian_deltas: list[np.ndarray] = []
    observed_cartesian_deltas: list[np.ndarray] = []
    commanded_pose = None
    if integrate_deltas_from_command:
        if episode.action_space not in {"delta_tquat", "delta_xyzrpy"}:
            raise ValueError("Integrated command replay is only valid for delta_tquat or delta_xyzrpy")
        import rcs

        # The integrated reference must start at the robot's measured reset pose.
        # Starting from zero or from the dataset action would produce an invalid
        # absolute Cartesian target on the first command.
        initial_tquat = np.asarray(observation[ROBOT_KEY]["tquat"], dtype=np.float64).copy()
        if initial_tquat.shape != (7,) or not np.isfinite(initial_tquat).all():
            raise ValueError(f"Invalid reset tquat for integrated replay: {initial_tquat}")
        commanded_pose = rcs.common.Pose(
            translation=initial_tquat[:3].reshape(3, 1),
            quaternion=initial_tquat[3:].reshape(4, 1),
        )
    interrupted = False

    try:
        for local_index, action_vector in enumerate(episode.actions):
            # This is deliberately sampled before action[t]: LMDB observation[t]
            # was recorded at the same point in the original control loop.
            actual_states.append(_state_from_observation(observation))
            wall_times.append(perf_counter() - start_time)
            if integrate_deltas_from_command:
                previous_commanded_pose = commanded_pose
                action, commanded_pose = _integrated_delta_action(
                    action_vector, episode.action_space, commanded_pose
                )
                if episode.action_space == "delta_xyzrpy":
                    # This is already the exact command supplied by the
                    # dataset. Avoid converting the integrated pose back to
                    # Euler angles, which can reintroduce branch artifacts.
                    commanded_delta = np.asarray(action_vector[:-1], dtype=np.float64).copy()
                else:
                    commanded_delta = np.concatenate(
                        [
                            np.asarray(commanded_pose.translation()).reshape(-1)
                            - np.asarray(previous_commanded_pose.translation()).reshape(-1),
                            _quaternion_to_rpy(
                                _quaternion_multiply(
                                    commanded_pose.rotation_q(),
                                    previous_commanded_pose.rotation_q()
                                    * np.array([-1.0, -1.0, -1.0, 1.0]),
                                )
                            ),
                        ]
                    )
            else:
                action = _action_for_environment(action_vector, episode.action_space, observation)
            previous_observation = observation
            observation, _, terminated, truncated, _ = env.step(action)
            if episode.action_space != "joints":
                commanded_cartesian_deltas.append(
                    commanded_delta
                    if integrate_deltas_from_command
                    else _cartesian_command_delta(action, episode.action_space)
                )
                observed_cartesian_deltas.append(
                    _observed_cartesian_delta(previous_observation, observation)
                )
            rate()
            if terminated or truncated:
                LOGGER.warning("Environment ended replay after selected frame %d", local_index)
                break
    except KeyboardInterrupt:
        interrupted = True
        LOGGER.warning("Replay interrupted by user; saving the partial trace")

    return ReplayTrace(
        actual_states=np.asarray(actual_states, dtype=np.float64).reshape(-1, STATE_DIM),
        wall_times=np.asarray(wall_times, dtype=np.float64),
        commanded_cartesian_deltas=np.asarray(commanded_cartesian_deltas, dtype=np.float64).reshape(-1, 6),
        observed_cartesian_deltas=np.asarray(observed_cartesian_deltas, dtype=np.float64).reshape(-1, 6),
        completed=len(actual_states),
        interrupted=interrupted,
    )


def _error_summary(error: np.ndarray) -> dict[str, Any]:
    abs_error = np.abs(error)
    return {
        "rmse_rad": float(np.sqrt(np.mean(np.square(error)))),
        "mae_rad": float(np.mean(abs_error)),
        "max_abs_rad": float(np.max(abs_error)),
        "per_joint_rmse_rad": np.sqrt(np.mean(np.square(error), axis=0)).tolist(),
        "per_joint_mae_rad": np.mean(abs_error, axis=0).tolist(),
        "per_joint_max_abs_rad": np.max(abs_error, axis=0).tolist(),
        "rmse_deg": float(np.rad2deg(np.sqrt(np.mean(np.square(error))))),
        "mae_deg": float(np.rad2deg(np.mean(abs_error))),
        "max_abs_deg": float(np.rad2deg(np.max(abs_error))),
        "per_joint_rmse_deg": np.rad2deg(np.sqrt(np.mean(np.square(error), axis=0))).tolist(),
    }


def _cartesian_tracking_summary(
    commanded: np.ndarray, observed: np.ndarray
) -> dict[str, Any]:
    error = observed - commanded
    commanded_cumulative = np.sum(commanded, axis=0)
    observed_cumulative = np.sum(observed, axis=0)
    cumulative_error = observed_cumulative - commanded_cumulative
    labels = ("x", "y", "z", "roll", "pitch", "yaw")
    return {
        "axes": labels,
        "commanded_cumulative": commanded_cumulative.tolist(),
        "observed_cumulative": observed_cumulative.tolist(),
        "cumulative_error": cumulative_error.tolist(),
        "commanded_translation_norm": float(np.linalg.norm(commanded_cumulative[:3])),
        "observed_translation_norm": float(np.linalg.norm(observed_cumulative[:3])),
        "commanded_rotation_norm_rad": float(np.linalg.norm(commanded_cumulative[3:])),
        "observed_rotation_norm_rad": float(np.linalg.norm(observed_cumulative[3:])),
        "commanded_rotation_norm_deg": float(np.rad2deg(np.linalg.norm(commanded_cumulative[3:]))),
        "observed_rotation_norm_deg": float(np.rad2deg(np.linalg.norm(observed_cumulative[3:]))),
        "commanded_translation_path_length": float(np.linalg.norm(commanded[:, :3], axis=1).sum()),
        "observed_translation_path_length": float(np.linalg.norm(observed[:, :3], axis=1).sum()),
        "commanded_rotation_path_length_rad": float(np.linalg.norm(commanded[:, 3:], axis=1).sum()),
        "observed_rotation_path_length_rad": float(np.linalg.norm(observed[:, 3:], axis=1).sum()),
        "commanded_rotation_path_length_deg": float(np.rad2deg(np.linalg.norm(commanded[:, 3:], axis=1).sum())),
        "observed_rotation_path_length_deg": float(np.rad2deg(np.linalg.norm(observed[:, 3:], axis=1).sum())),
        "increment_error": {
            label: {
                "rmse": float(np.sqrt(np.mean(np.square(error[:, axis])))),
                "mae": float(np.mean(np.abs(error[:, axis]))),
                "max_abs": float(np.max(np.abs(error[:, axis]))),
            }
            for axis, label in enumerate(labels)
        },
    }


def compute_metrics(episode: Episode, trace: ReplayTrace, replay_fps: float) -> dict[str, Any]:
    count = trace.completed
    if count == 0:
        msg = "No live states were recorded"
        raise ValueError(msg)
    recorded = episode.observations[:count]
    actions = episode.actions[:count]
    actual = trace.actual_states
    joint_error = actual[:, :ARM_DOF] - recorded[:, :ARM_DOF]
    gripper_error = actual[:, ARM_DOF] - recorded[:, ARM_DOF]

    metrics: dict[str, Any] = {
        "episode": episode.index,
        "task": episode.task,
        "action_space": episode.action_space,
        "dataset_fps": episode.fps,
        "replay_fps": replay_fps,
        "selected_start_frame": int(episode.frame_indices[0]),
        "selected_stop_frame_exclusive": int(episode.frame_indices[-1] + 1),
        "requested_frames": len(episode.actions),
        "completed_frames": count,
        "interrupted": trace.interrupted,
        "joint_observation_tracking": _error_summary(joint_error),
        "gripper_observation_tracking": {
            "rmse": float(np.sqrt(np.mean(np.square(gripper_error)))),
            "mae": float(np.mean(np.abs(gripper_error))),
            "max_abs": float(np.max(np.abs(gripper_error))),
        },
        # This is an instantaneous pre-command delta, not a settled controller error.
    }
    if episode.action_space == "joints":
        metrics["joint_pre_command_delta"] = _error_summary(actual[:, :ARM_DOF] - actions[:, :ARM_DOF])
    elif episode.action_space == "delta_xyzrpy":
        cumulative = np.sum(actions[:, :6], axis=0)
        metrics["cumulative_delta_xyzrpy"] = {
            "translation": cumulative[:3].tolist(),
            "translation_norm": float(np.linalg.norm(cumulative[:3])),
            "rotation_rpy": cumulative[3:].tolist(),
            "rotation_norm_rad": float(np.linalg.norm(cumulative[3:])),
            "rotation_norm_deg": float(np.rad2deg(np.linalg.norm(cumulative[3:]))),
        }
    if episode.action_space != "joints" and len(trace.commanded_cartesian_deltas) > 0:
        metrics["cartesian_tracking"] = _cartesian_tracking_summary(
            trace.commanded_cartesian_deltas,
            trace.observed_cartesian_deltas,
        )
    if count > 1:
        elapsed = trace.wall_times[-1] - trace.wall_times[0]
        metrics["measured_sample_fps"] = float((count - 1) / elapsed) if elapsed > 0 else None
    else:
        metrics["measured_sample_fps"] = None
    return metrics


def save_results(
    output_dir: Path,
    episode: Episode,
    trace: ReplayTrace,
    metrics: dict[str, Any],
) -> tuple[Path, Path, Path, Path | None]:
    """Save raw aligned arrays, metrics, and replay plots."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stem = f"rumi_pnp_episode_{episode.index:03d}_{stamp}"
    npz_path = output_dir / f"{stem}.npz"
    json_path = output_dir / f"{stem}_metrics.json"
    plot_path = output_dir / f"{stem}_joint_tracking.png"
    cartesian_plot_path = (
        output_dir / f"{stem}_cartesian_tracking.png" if episode.action_space != "joints" else None
    )

    count = trace.completed
    recorded = episode.observations[:count]
    actions = episode.actions[:count]
    actual = trace.actual_states
    error = actual[:, :ARM_DOF] - recorded[:, :ARM_DOF]
    time_axis = episode.timestamps[:count] - episode.timestamps[0]

    np.savez_compressed(
        npz_path,
        global_indices=episode.global_indices[:count],
        frame_indices=episode.frame_indices[:count],
        dataset_timestamps=episode.timestamps[:count],
        wall_times=trace.wall_times,
        recorded_observations=recorded,
        recorded_actions=actions,
        actual_observations=actual,
        joint_observation_error=error,
        commanded_cartesian_deltas=trace.commanded_cartesian_deltas,
        observed_cartesian_deltas=trace.observed_cartesian_deltas,
    )
    json_path.write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")

    fig, axes = plt.subplots(4, 2, figsize=(15, 13), sharex=True, constrained_layout=True)
    axes_flat = axes.ravel()
    for joint in range(ARM_DOF):
        axis = axes_flat[joint]
        axis.plot(time_axis, recorded[:, joint], label="LMDB observation", linewidth=1.8)
        axis.plot(time_axis, actual[:, joint], label="actual observation", linewidth=1.2)
        if episode.action_space == "joints":
            axis.plot(time_axis, actions[:, joint], label="LMDB action", linewidth=0.9, linestyle=":", alpha=0.8)
        joint_rmse_deg = np.rad2deg(np.sqrt(np.mean(error[:, joint] ** 2)))
        axis.set_title(f"Joint {joint + 1} — RMSE {joint_rmse_deg:.3f} deg")
        axis.set_ylabel("position [rad]")
        axis.grid(alpha=0.3)

    error_axis = axes_flat[-1]
    for joint in range(ARM_DOF):
        error_axis.plot(time_axis, np.rad2deg(np.abs(error[:, joint])), label=f"J{joint + 1}", linewidth=1.0)
    error_axis.set_title("Absolute actual-vs-LMDB observation error")
    error_axis.set_ylabel("absolute error [deg]")
    error_axis.grid(alpha=0.3)
    error_axis.legend(ncol=4, fontsize=8)
    for axis in axes[-1, :]:
        axis.set_xlabel("trajectory time [s]")
    axes_flat[0].legend(ncol=3, fontsize=8)
    fig.suptitle(
        f"RUMI PnP episode {episode.index}: actual robot tracking quality " f"({count}/{len(episode.actions)} frames)",
        fontsize=14,
    )
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    if cartesian_plot_path is not None:
        commanded = trace.commanded_cartesian_deltas
        observed = trace.observed_cartesian_deltas
        cartesian_time = trace.wall_times[: len(commanded)]
        labels = ("x", "y", "z", "roll", "pitch", "yaw")
        units = ("m", "m", "m", "rad", "rad", "rad")
        cartesian_fig, cartesian_axes = plt.subplots(
            3, 2, figsize=(15, 10), sharex=True, constrained_layout=True
        )
        for axis_index, axis in enumerate(cartesian_axes.ravel()):
            axis.plot(cartesian_time, commanded[:, axis_index], label="commanded", linewidth=1.2)
            axis.plot(cartesian_time, observed[:, axis_index], label="observed", linewidth=1.2)
            axis.set_title(f"{labels[axis_index]} increment")
            axis.set_ylabel(f"{labels[axis_index]} [{units[axis_index]}]")
            axis.grid(alpha=0.3)
        for axis in cartesian_axes[-1, :]:
            axis.set_xlabel("wall time [s]")
        cartesian_axes[0, 0].legend()
        cartesian_fig.suptitle(
            f"RUMI episode {episode.index}: commanded vs observed Cartesian increments",
            fontsize=14,
        )
        cartesian_fig.savefig(cartesian_plot_path, dpi=160)
        plt.close(cartesian_fig)

    return npz_path, json_path, plot_path, cartesian_plot_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET, help="Single-file rcs.lmdb_joint.v1 dataset")
    parser.add_argument("--episode", type=int, default=0, help="Zero-based LMDB episode index")
    parser.add_argument("--start", type=int, default=0, help="First episode-local frame to replay")
    parser.add_argument("--stop", type=int, default=None, help="Exclusive episode-local stop frame")
    parser.add_argument("--fps", type=float, default=None, help="Replay FPS; defaults to LMDB metadata FPS")
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--gripper-type", choices=("robotiq", "franka"), default="robotiq")
    parser.add_argument(
        "--max-joint-step-deg",
        type=float,
        default=DEFAULT_MAX_JOINT_STEP_DEG,
        help="Per-step safety limit used by LimitedAbsoluteAction; <=0 disables it",
    )
    parser.add_argument("--speed-factor", type=float, default=0.1, help="Franka reset/home speed factor")
    parser.add_argument(
        "--action-space",
        choices=ACTION_SPACES,
        default="joints",
        help=(
            "Action representation stored in the LMDB. Cartesian tquat actions use Cartesian "
            "Relative TQuat control; XYZ-RPY actions use Cartesian Relative TRPY control."
        ),
    )
    parser.add_argument(
        "--integrate-deltas-from-command",
        action="store_true",
        help=(
            "For delta actions, integrate from the last commanded Cartesian pose and send absolute targets; "
            "valid only for delta_tquat and delta_xyzrpy."
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--inspect-only", action="store_true", help="Load and describe the selection without moving hardware"
    )
    parser.add_argument("--yes", action="store_true", help="Skip the final interactive motion confirmation")
    return parser.parse_args()


def _print_selection(episode: Episode, replay_fps: float) -> None:
    duration = len(episode.actions) / replay_fps
    initial = np.array2string(episode.observations[0, :ARM_DOF], precision=5, separator=", ")
    print(f"Dataset contains {episode.total_episodes} episodes; selected episode {episode.index} ({episode.task!r}).")
    print(
        f"Selected frames [{episode.frame_indices[0]}, {episode.frame_indices[-1] + 1}) of "
        f"{episode.full_episode_length}: {len(episode.actions)} commands at {replay_fps:g} Hz "
        f"(~{duration:.2f} s)."
    )
    print(f"Reset/home joint target [rad]: {initial}")
    if episode.action_space == "delta_xyzrpy":
        cumulative = np.sum(episode.actions[:, :6], axis=0)
        print(
            "Cumulative delta XYZ-RPY: "
            f"translation={np.array2string(cumulative[:3], precision=6)}, "
            f"rotation_rpy={np.array2string(cumulative[3:], precision=6)} "
            f"({np.rad2deg(np.linalg.norm(cumulative[3:])):.3f} deg norm)"
        )
    elif episode.action_space in {"xyzrpy", "tquat"}:
        motion = _cumulative_cartesian_motion(episode.actions, episode.action_space)
        print(
            f"Cumulative {episode.action_space} Cartesian motion: "
            f"translation={np.array2string(np.asarray(motion['translation']), precision=6)}, "
            f"translation_norm={motion['translation_norm']:.6f} m, "
            f"path_length={motion['translation_path_length']:.6f} m"
        )
        print(
            "  rotation_rpy="
            f"{np.array2string(np.asarray(motion['rotation_rpy']), precision=6)} "
            f"({motion['rotation_norm_deg']:.3f} deg norm), "
            f"path_length={motion['rotation_path_length_deg']:.3f} deg"
        )


def main() -> int:
    args = parse_args()
    logging.basicConfig(format="%(asctime)s - %(levelname)s - %(message)s", level=logging.INFO)
    episode = load_episode(args.dataset, args.episode, args.start, args.stop, args.action_space)
    if args.integrate_deltas_from_command and episode.action_space not in {"delta_tquat", "delta_xyzrpy"}:
        raise ValueError("--integrate-deltas-from-command requires --action-space delta_tquat or delta_xyzrpy")
    replay_fps = episode.fps if args.fps is None else args.fps
    if replay_fps <= 0:
        msg = f"fps must be positive, got {replay_fps}"
        raise ValueError(msg)
    if not 0 < args.speed_factor <= 1:
        msg = f"speed-factor must be in (0, 1], got {args.speed_factor}"
        raise ValueError(msg)

    _print_selection(episode, replay_fps)
    if args.inspect_only:
        return 0

    if not args.yes:
        answer = input(
            "WARNING: this will move the physical robot and gripper. Clear the workspace, hold the E-stop, "
            "and type 'replay' to continue: "
        ).strip()
        if answer != "replay":
            print("Replay cancelled.")
            return 1

    env = build_hardware_env(
        initial_joints=episode.observations[0, :ARM_DOF],
        robot_ip=args.robot_ip,
        gripper_type=args.gripper_type,
        max_joint_step_deg=args.max_joint_step_deg,
        speed_factor=args.speed_factor,
        action_space=args.action_space,
        integrate_deltas_from_command=args.integrate_deltas_from_command,
    )
    with env:
        trace = replay(
            env,
            episode,
            replay_fps,
            integrate_deltas_from_command=args.integrate_deltas_from_command,
        )

    metrics = compute_metrics(episode, trace, replay_fps)
    npz_path, json_path, plot_path, cartesian_plot_path = save_results(
        args.output_dir, episode, trace, metrics
    )
    tracking = metrics["joint_observation_tracking"]
    print(
        "Joint observation tracking: "
        f"RMSE={tracking['rmse_deg']:.3f} deg, MAE={tracking['mae_deg']:.3f} deg, "
        f"max={tracking['max_abs_deg']:.3f} deg"
    )
    print(f"Raw trace: {npz_path}")
    print(f"Metrics:   {json_path}")
    print(f"Plot:      {plot_path}")
    if cartesian_plot_path is not None:
        print(f"Cartesian: {cartesian_plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
