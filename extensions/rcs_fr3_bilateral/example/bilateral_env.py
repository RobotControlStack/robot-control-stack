import argparse
import time

import numpy as np
from rcs_fr3_bilateral._core import hw
from rcs_fr3_bilateral.configs import DefaultFR3BilateralTeleop


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start the RCS FR3 bilateral teleoperation wrapper.")
    parser.add_argument("--leader-ip", default="192.168.102.1")
    parser.add_argument("--follower-ip", default="192.168.101.1")
    parser.add_argument("--update-rate-hz", type=float, default=1000.0)
    parser.add_argument(
        "--move-home",
        action="store_true",
        help="Move both robots to their configured q_home before starting teleoperation.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Run duration in seconds. 0 means run until Ctrl-C.",
    )
    parser.add_argument(
        "--mode",
        choices=("gravity_only", "bilateral"),
        default="gravity_only",
        help="gravity_only exchanges state data while both robots stay in zero-torque guiding.",
    )
    parser.add_argument(
        "--disable-leader-haptics",
        action="store_true",
        help="In bilateral mode, leave the leader in zero-torque guiding instead of applying haptic torque.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    cfg = DefaultFR3BilateralTeleop(
        leader_ip=args.leader_ip,
        follower_ip=args.follower_ip,
        update_rate_hz=args.update_rate_hz,
        gravity_only=args.mode == "gravity_only",
        ignore_realtime=True,
        leader_haptic_feedback=not args.disable_leader_haptics,
    ).config()

    teleop = hw.BilateralFranka(cfg)
    mode_name = "gravity_only" if cfg.control_mode == hw.BilateralControlMode.gravity_only else "bilateral"
    print(f"Starting bilateral FR3 setup in {mode_name} mode.")
    print("Press Ctrl-C to stop.")

    start_time = time.monotonic()
    try:
        if args.move_home:
            print("Moving both robots to their configured home poses.")
            teleop.move_home()
        teleop.start()
        while args.duration <= 0.0 or time.monotonic() - start_time < args.duration:
            state = teleop.get_state()
            leader_follower_error = np.linalg.norm(state.leader_q - state.follower_q)
            follower_tau_norm = np.linalg.norm(state.follower_external_tau)
            leader_cmd_norm = np.linalg.norm(state.leader_torque_command)
            print(
                "running=%s leader/follower_q_norm_delta=%.4f follower_tau_ext_norm=%.4f leader_cmd_norm=%.4f"
                % (
                    state.running,
                    leader_follower_error,
                    follower_tau_norm,
                    leader_cmd_norm,
                )
            )
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping bilateral FR3 setup.")
    finally:
        teleop.stop()


if __name__ == "__main__":
    main()
