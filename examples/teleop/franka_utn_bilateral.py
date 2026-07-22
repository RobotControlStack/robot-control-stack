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
from rcs.envs.base import HardwareEnv
from rcs.envs.storage_wrapper import StorageWrapper
from rcs.operator.pedals import FootPedal
from rcs.utils import SimpleFrameRate
from rcs_fr3._core import hw as fr3_hw
from rcs_fr3.configs import DefaultFR3HardwareEnv
from rcs_fr3_bilateral import BilateralFR3Wrapper
from rcs_fr3_bilateral._core import hw
from rcs_fr3_bilateral.configs import DefaultFR3BilateralTeleop

import rcs


LEADER_IP = "192.168.102.1"
FOLLOWER_IP = "192.168.101.1"
DEFAULT_STACK_FREQUENCY_HZ = 30.0
DEFAULT_CONTROL_FREQUENCY_HZ = 1000.0
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


def make_env(args: argparse.Namespace) -> tuple[StorageWrapper | BilateralFR3Wrapper, hw.BilateralFranka]:
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
    config.follower_cfg.tcp_offset = rcs.GRIPPER_TCP_OFFSETS[GripperType(args.gripper_type)]
    teleop = hw.BilateralFranka(config)

    # BilateralFR3Wrapper replaces RobotWrapper here.  In particular, do not
    # add a RobotWrapper for teleop.get_follower(), since that would issue
    # low-rate commands concurrently with the bilateral controller.
    env: StorageWrapper | BilateralFR3Wrapper = BilateralFR3Wrapper(HardwareEnv(), teleop)
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
    return env, teleop


def leader_action(env: StorageWrapper | BilateralFR3Wrapper, gripper_closed: bool) -> dict[str, dict[str, np.ndarray]]:
    """Produce a stack-rate action placeholder and preserve it for recording.

    ``BilateralFR3Wrapper`` replaces this value with its cached previous
    leader sample before it reaches the inner environment.  Passing the
    current sample here lets an outer StorageWrapper retain both the current
    ``env_action`` and the previous-step ``action`` in its usual format.
    """
    wrapper = env.get_wrapper_attr("get_leader_action")
    action = wrapper()
    action["right"]["gripper"] = np.array([0.0 if gripper_closed else 1.0], dtype=np.float32)
    return action


def make_follower_gripper(robot_ip: str, serial_number: str, gripper_type: GripperType) -> Gripper:
    """Create a follower gripper using the single-arm FR3 configuration pattern.

    ``FrankaHand`` uses the standard FR3 hardware configuration for the
    follower IP.  ``Robotiq2F85`` uses its USB serial-number configuration.
    Both are asynchronous, so pedal commands never stall the stack loop.
    """
    if gripper_type.id == GripperType.FrankaHand.id:
        default_env = DefaultFR3HardwareEnv()
        default_env.ip = robot_ip
        gripper_cfg = default_env.config().gripper_cfg
        if not isinstance(gripper_cfg, fr3_hw.FHConfig):
            raise TypeError(f"Expected an FHConfig, got {type(gripper_cfg).__name__}.")
        gripper_cfg.async_control = True
        return fr3_hw.FrankaHand(gripper_cfg)

    if gripper_type.id == FOLLOWER_GRIPPER_TYPE.id:
        try:
            from rcs_robotiq2f85.hw import RobotiQ2F85Gripper, RobotiQ2F85GripperConfig
        except ImportError as exc:
            raise ImportError("Robotiq support requires the rcs_robotiq2f85 extension.") from exc
        gripper_cfg = RobotiQ2F85GripperConfig(
            serial_number=serial_number,
            speed=100,
            force=50,
            async_control=True,
        )
        return RobotiQ2F85Gripper(gripper_cfg)

    raise ValueError(f"Unsupported follower gripper type: {gripper_type.id}")


def update_gripper_from_pedal(pedal: FootPedal, gripper: Gripper, was_closed: bool | None) -> bool:
    """Apply the held-to-close semantics of pedal key B.

    ``KEY_B`` pressed/held closes the follower gripper; releasing it opens the
    gripper.  The A/B/C constants make the physical pedal mapping explicit,
    even though only B is currently assigned a command.
    """
    _ = (PEDAL_KEY_A, PEDAL_KEY_C)
    is_closed = pedal.get_key_state(PEDAL_KEY_B)
    if is_closed != was_closed:
        if is_closed:
            gripper.grasp()
        else:
            gripper.open()
    return is_closed


def main() -> None:
    args = parse_args()
    if args.stack_frequency_hz <= 0 or args.control_frequency_hz <= 0:
        raise ValueError("Both frequencies must be positive.")

    env, teleop = make_env(args)
    rate_limiter = SimpleFrameRate(args.stack_frequency_hz, "bilateral RCS stack loop")
    pedal: FootPedal | None = None
    gripper: Gripper | None = None
    try:
        pedal = FootPedal("FootSwitch Keyboard")
        gripper = make_follower_gripper(
            robot_ip=args.follower_ip,
            serial_number=args.gripper_serial,
            gripper_type=GripperType(args.gripper_type),
        )
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
            pedal_gripper_closed: bool | None = None
            while True:
                assert pedal is not None and gripper is not None
                pedal_gripper_closed = update_gripper_from_pedal(pedal, gripper, pedal_gripper_closed)
                _, _, terminated, truncated, info = env.step(leader_action(env, pedal_gripper_closed))
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
        if teleop.is_running():
            teleop.stop()
        if gripper is not None:
            gripper.close()
        if pedal is not None:
            pedal.close()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
