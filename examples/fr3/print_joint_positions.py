"""Print the current joint positions of a Franka Research 3 robot."""

from __future__ import annotations

import argparse
import sys

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read and print the current FR3 joint positions.",
    )
    parser.add_argument("ip", help="Robot hostname or IP address")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        from rcs_fr3._core import hw

        robot = hw.Franka(hw.FR3Config(ip=args.ip, ignore_realtime=True))
        q_rad = np.asarray(robot.get_joint_position(), dtype=np.float64)
    except Exception as exc:
        print(f"Failed to read joint positions from {args.ip}: {exc}", file=sys.stderr)
        print(
            "Make sure the robot is reachable, unlocked, FCI is active, and no "
            "other process is connected through FCI.",
            file=sys.stderr,
        )
        return 1

    q_deg = np.rad2deg(q_rad)

    print(f"FR3 joint positions at {args.ip}:")
    print(" joint       radians       degrees")
    for index, (radians, degrees) in enumerate(zip(q_rad, q_deg, strict=True), start=1):
        print(f" J{index:<2}    {radians:+11.6f}    {degrees:+11.3f}")

    print("\nq [rad] = [" + ", ".join(f"{value:.8f}" for value in q_rad) + "]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
