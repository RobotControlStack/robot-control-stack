"""Run UTN's bilateral FR3 teleoperation through the RCS wrapper stack.

The bilateral backend owns both Frankas and controls them at 1000 Hz.  This
script only steps the RCS environment at the requested recording/observation
rate (30 Hz by default).  At each RCS step, the wrapper exposes the latest
follower observation and uses the leader joint position from the previous RCS
step as the stack action.

Put both robots in FCI mode before running this script.  ``--move-home`` is
opt-in because it physically moves both robots.
"""

from __future__ import annotations

import argparse
import logging

import numpy as np
from rcs._core.common import Gripper, GripperType
from rcs.envs.base import GripperWrapper, HardwareEnv, MultiRobotWrapper
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.pedals import FootPedal
from rcs.utils import SimpleFrameRate
from rcs_fr3.configs import SingleArmFR3MultiHardwareEnv
from rcs_fr3.creators import HARDWARE_GRIPPER_CREATORS
from rcs_fr3_bilateral import BilateralFR3Wrapper
from rcs_fr3_bilateral._core import hw
from rcs_fr3_bilateral.configs import DefaultFR3BilateralTeleop


LEADER_IP = "192.168.102.1"
FOLLOWER_IP = "192.168.101.1"
DEFAULT_STACK_FREQUENCY_HZ = 30.0
DEFAULT_CONTROL_FREQUENCY_HZ = 1000.0
DEFAULT_BINARY_GRIPPER = False
FOLLOWER_GRIPPER_TYPE = GripperType("Robotiq2F85")
# The follower is the FR3 at 192.168.101.1 (the left-side hardware in the
# standard FR3 duo mapping). Override this if its USB gripper differs.
DEFAULT_FOLLOWER_GRIPPER_SERIAL = "DAAQMJHX"

# The pedal advertises its three switches as keyboard-style key events.  A
# and C are reserved for future teleop controls; B is the follower gripper.
PEDAL_KEY_A = "KEY_A"
PEDAL_KEY_B = "KEY_B"
PEDAL_KEY_C = "KEY_C"

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run bilateral FR3 teleoperation in the RCS environment stack.")
    parser.add_argument("--leader-ip", default=LEADER_IP)
    parser.add_argument("--follower-ip", default=FOLLOWER_IP)
    parser.add_argument("--stack-frequency-hz", type=float, default=DEFAULT_STACK_FREQUENCY_HZ)
    parser.add_argument("--control-frequency-hz", type=float, default=DEFAULT_CONTROL_FREQUENCY_HZ)
    parser.add_argument(
        "--gripper-type",
        choices=(GripperType.FrankaHand.id, FOLLOWER_GRIPPER_TYPE.id),
        default=FOLLOWER_GRIPPER_TYPE.id,
        help="Follower end effector used for pedal-controlled grasping.",
    )
    parser.add_argument("--gripper-serial", default=DEFAULT_FOLLOWER_GRIPPER_SERIAL)
    parser.add_argument(
        "--binary-gripper",
        action="store_true",
        default=DEFAULT_BINARY_GRIPPER,
        help="Use binary gripper observations/actions instead of the default normalized-width observation.",
    )
    parser.add_argument("--record-dir", type=str, default="test_bilateral", help="Optional Parquet dataset directory.")
    parser.add_argument("--instruction", default="bilateral FR3 teleoperation")
    parser.add_argument(
        "--mode",
        choices=("gravity_only", "bilateral"),
        default="bilateral",
        help="Use gravity-only validation or active bilateral follower tracking.",
    )
    parser.add_argument("--disable-leader-haptics", action="store_true")
    return parser.parse_args()


def make_env(
    args: argparse.Namespace, gripper: Gripper, follower_robot_cfg: object
) -> tuple[StorageWrapper | MultiRobotWrapper, hw.BilateralFranka, BilateralFR3Wrapper]:
    config = DefaultFR3BilateralTeleop(
        leader_ip=args.leader_ip,
        follower_ip=args.follower_ip,
        gravity_only=args.mode == "gravity_only",
        # The backend already owns the 1 kHz control loop.  Do not let a
        # lower-rate RCS environment attempt real-time robot control.
        ignore_realtime=True,
        update_rate_hz=args.control_frequency_hz,
        leader_haptic_feedback=not args.disable_leader_haptics,
    ).config()
    config.follower_cfg.tcp_offset = follower_robot_cfg.tcp_offset  # type: ignore[attr-defined]
    config.follower_cfg.q_home = follower_robot_cfg.q_home  # type: ignore[attr-defined]
    teleop = hw.BilateralFranka(config)

    # Match the standard single-arm hardware stack. BilateralFR3Wrapper
    # replaces RobotWrapper, while GripperWrapper remains responsible for the
    # gripper command and observation. MultiRobotWrapper provides the
    # top-level ``right`` nesting used by the teleop dataset.
    bilateral_env = BilateralFR3Wrapper(HardwareEnv(), teleop)
    robot_env = GripperWrapper(bilateral_env, gripper, binary=args.binary_gripper)
    env: StorageWrapper | MultiRobotWrapper = MultiRobotWrapper({"right": robot_env})
    if args.record_dir:
        env = StorageWrapper(
            env,
            args.record_dir,
            instruction=args.instruction,
            always_record=True,
            batch_size=32,
            max_rows_per_group=100,
            max_rows_per_file=1000,
        )
    return env, teleop, bilateral_env


def leader_action(bilateral_env: BilateralFR3Wrapper, gripper_closed: bool) -> dict[str, dict[str, np.ndarray]]:
    """Produce a stack-rate action placeholder and preserve it for recording.

    ``BilateralFR3Wrapper`` replaces this value with its cached previous
    leader sample before it reaches the inner environment.  Passing the
    current sample here lets an outer StorageWrapper retain both the current
    ``env_action`` and the previous-step ``action`` in its usual format.
    """
    return {
        "right": {
            **bilateral_env.get_leader_action(),
            "gripper": np.array([0.0 if gripper_closed else 1.0], dtype=np.float32),
        }
    }


def make_follower_hardware(args: argparse.Namespace) -> tuple[object, Gripper]:
    """Build the follower robot/gripper configuration exactly as UTN teleop does."""
    creator = SingleArmFR3MultiHardwareEnv()
    creator.gripper_serial_number = args.gripper_serial
    cfg = creator.config(
        grippertype=GripperType(args.gripper_type),
        robot_ip=args.follower_ip,
    )
    robot_cfg = cfg.robot_cfgs["right"]
    if cfg.gripper_cfgs is None:
        raise RuntimeError("Single-arm FR3 configuration did not provide follower gripper configurations.")
    gripper_cfg = cfg.gripper_cfgs["right"]
    if gripper_cfg is None:
        raise RuntimeError("Single-arm FR3 configuration did not provide a follower gripper.")
    gripper_type_id = gripper_cfg.gripper_type.id
    if gripper_type_id not in HARDWARE_GRIPPER_CREATORS:
        raise ValueError(f"Unsupported follower gripper type: {gripper_type_id}")
    return robot_cfg, HARDWARE_GRIPPER_CREATORS[gripper_type_id](gripper_cfg)


def gripper_command_from_pedal(pedal: FootPedal) -> bool:
    """Return the held-to-close command for pedal key B.

    ``KEY_B`` pressed/held maps to a closed command; releasing it maps to an
    open command. GripperWrapper sends that command to the physical gripper.
    """
    _ = (PEDAL_KEY_A, PEDAL_KEY_C)
    return pedal.get_key_state(PEDAL_KEY_B)


def main() -> None:
    args = parse_args()
    if args.stack_frequency_hz <= 0 or args.control_frequency_hz <= 0:
        raise ValueError("Both frequencies must be positive.")

    rate_limiter = SimpleFrameRate(args.stack_frequency_hz, "bilateral RCS stack loop")
    pedal: FootPedal | None = None
    gripper: Gripper | None = None
    teleop: hw.BilateralFranka | None = None
    try:
        pedal = FootPedal("FootSwitch Keyboard")
        follower_robot_cfg, gripper = make_follower_hardware(args)
        env, teleop, bilateral_env = make_env(args, gripper, follower_robot_cfg)
        logger.warning("Moving both robots to q_home.")
        teleop.move_home()
        with env:
            env.reset()
            logger.info(
                "Bilateral teleoperation started: leader=%s follower=%s, stack=%.1f Hz, control=%.1f Hz",
                args.leader_ip,
                args.follower_ip,
                args.stack_frequency_hz,
                args.control_frequency_hz,
            )
            while True:
                assert pedal is not None and gripper is not None
                pedal_gripper_closed = gripper_command_from_pedal(pedal)
                _, _, terminated, truncated, info = env.step(leader_action(bilateral_env, pedal_gripper_closed))
                if terminated or truncated:
                    logger.warning("Environment ended: terminated=%s truncated=%s", terminated, truncated)
                    break
                if not info["right"]["bilateral_follower_state"]["running"]:
                    raise RuntimeError("Bilateral controller stopped unexpectedly.")
                rate_limiter()
    except KeyboardInterrupt:
        logger.info("Stopping bilateral teleoperation.")
    finally:
        # StorageWrapper currently closes only its writer thread, not its
        # wrapped environment.  Stop explicitly so this remains safe whether
        # recording is enabled or not.
        if teleop is not None and teleop.is_running():
            teleop.stop()
        if gripper is not None:
            gripper.close()
        if pedal is not None:
            pedal.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
