import argparse
import logging
import os
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from rcs.envs.base import CoverWrapper, GripperWrapper, HardwareEnv

from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig

logger = logging.getLogger(__name__)


ROBOTIQ_SERIAL_ENV_VAR = "ROBOTIQ_2F85_SERIAL_NUMBER"
ROBOTIQ_SERIAL_NUMBER = "DAAQMJHX"
DEFAULT_SPEED_MM_S = 100.0
DEFAULT_FORCE_N = 20
DEFAULT_ASYNC_CONTROL = True
DEFAULT_BINARY_ACTIONS = False


@dataclass(frozen=True)
class GripperOnlyConfig:
    serial_number: str
    speed: float = DEFAULT_SPEED_MM_S
    force: float = DEFAULT_FORCE_N
    async_control: bool = DEFAULT_ASYNC_CONTROL
    binary_actions: bool = DEFAULT_BINARY_ACTIONS


class GripperOnlyHardwareEnv(HardwareEnv):
    """Minimal hardware env whose only controllable device is a gripper."""

    metadata = {"render_modes": []}

    def __init__(self) -> None:
        self.action_space = gym.spaces.Dict({})
        self.observation_space = gym.spaces.Dict({})
        super().__init__()


def _resolve_serial_number(serial_number: str | None = None) -> str:
    resolved = serial_number or os.getenv(ROBOTIQ_SERIAL_ENV_VAR) or ROBOTIQ_SERIAL_NUMBER
    if not resolved:
        msg = (
            "Robotiq 2F85 serial number is required. Pass --serial-number, set "
            f"{ROBOTIQ_SERIAL_ENV_VAR}, or edit ROBOTIQ_SERIAL_NUMBER in this file."
        )
        raise ValueError(msg)
    return resolved


def make_gripper(cfg: GripperOnlyConfig) -> RobotiQ2F85Gripper:
    gripper_cfg = RobotiQ2F85GripperConfig(
        serial_number=cfg.serial_number,
        speed=cfg.speed,
        force=cfg.force,
        async_control=cfg.async_control,
    )
    return RobotiQ2F85Gripper(gripper_cfg)


def get_env(
    serial_number: str | None = None,
    *,
    speed: float = DEFAULT_SPEED_MM_S,
    force: float = DEFAULT_FORCE_N,
    async_control: bool = DEFAULT_ASYNC_CONTROL,
    binary_actions: bool = DEFAULT_BINARY_ACTIONS,
) -> gym.Env:
    cfg = GripperOnlyConfig(
        serial_number=_resolve_serial_number(serial_number),
        speed=speed,
        force=force,
        async_control=async_control,
        binary_actions=binary_actions,
    )

    logger.info("Initializing Robotiq 2F85 gripper with serial number %s", cfg.serial_number)
    gripper = make_gripper(cfg)
    env: gym.Env = GripperOnlyHardwareEnv()
    env = GripperWrapper(env, gripper, binary=cfg.binary_actions)
    return CoverWrapper(env)


def _status_dict(status: Any) -> dict[str, Any]:
    return {
        "activated": status.state.activated,
        "is_activated": status.state.is_activated,
        "moving": status.state.moving,
        "object_detected": status.state.obj_detected,
        "opening_mm": round(status.state.opening, 2),
        "goal_opening_mm": round(status.state.goal_opening, 2),
        "current_ma": round(status.state.current, 2),
        "fault": status.state.fault,
    }


def _print_status(env: gym.Env) -> None:
    gripper = env.get_wrapper_attr("gripper")
    status = gripper.get_state()
    logger.info("Gripper status: %s", _status_dict(status))


def _interactive_loop(env: gym.Env) -> None:
    obs, info = env.reset()
    logger.info("Environment reset: obs=%s info=%s", obs, info)
    _print_status(env)

    while True:
        cmd = input("gripper> ").strip().lower()
        if cmd in {"q", "quit", "exit"}:
            return
        if cmd in {"s", "status"}:
            _print_status(env)
            continue
        if cmd in {"r", "reset"}:
            obs, info = env.reset()
            logger.info("Environment reset: obs=%s info=%s", obs, info)
            continue
        force = None
        if cmd in {"o", "open"}:
            width = 1.0
        elif cmd in {"c", "close", "shut"}:
            width = 0.0
        else:
            parts = cmd.split()
            try:
                width = float(parts[0])
                force = float(parts[1]) if len(parts) == 2 else None
            except (IndexError, ValueError):
                logger.info(
                    "Commands: open, close, reset, status, quit, or a normalized width in [0, 1]. "
                    "Use '<width> <force>' to override force for one command."
                )
                continue

        width = float(np.clip(width, 0.0, 1.0))
        action = {"gripper": np.array([width], dtype=np.float32)}
        if force is not None:
            action["gripper_force"] = np.array([force], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        logger.info(
            "Commanded width %.3f: obs=%s reward=%s terminated=%s truncated=%s info=%s",
            width,
            obs,
            reward,
            terminated,
            truncated,
            info,
        )
        _print_status(env)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Initialize and control only the Robotiq 2F85 gripper.")
    parser.add_argument("--serial-number", default=None, help=f"Defaults to ${ROBOTIQ_SERIAL_ENV_VAR}.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MM_S, help="Command speed in mm/s.")
    parser.add_argument("--force", type=float, default=DEFAULT_FORCE_N, help="Command force in N.")
    parser.add_argument("--sync", action="store_true", help="Wait for each gripper movement to complete.")
    parser.add_argument("--binary", action="store_true", help="Round actions to binary open/closed commands.")
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    args = _parse_args()
    env = get_env(
        serial_number=args.serial_number,
        speed=args.speed,
        force=args.force,
        async_control=not args.sync,
        binary_actions=args.binary,
    )
    with env:
        _interactive_loop(env)


if __name__ == "__main__":
    main()
