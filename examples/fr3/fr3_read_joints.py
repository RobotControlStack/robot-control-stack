"""Read out the joint positions of a real Franka FR3 using rcs.

Prerequisites:
  * rcs_fr3 extension installed (`pip install -ve extensions/rcs_fr3 --no-build-isolation`)
  * the robot has its joints unlocked and FCI active, e.g. via
        python -m rcs_fr3 unlock 192.168.12.1

Usage:
    python examples/fr3/fr3_read_joints.py
"""

import time

import numpy as np
from rcs_fr3 import hw
from rcs_fr3.configs import DefaultFR3HardwareEnv

ROBOT_IP = "192.168.1.12"
RATE_HZ = 10.0


def main() -> None:
    # Reuse the default hardware config; only the robot part is needed (no gripper, no cameras).
    env_creator = DefaultFR3HardwareEnv()
    env_creator.ip = ROBOT_IP
    robot_cfg = env_creator.config().robot_cfg
    robot_cfg.ignore_realtime = True  # reading state does not need a realtime kernel

    robot = hw.Franka(robot_cfg)
    np.set_printoptions(precision=4, suppress=True)
    try:
        q = robot.get_joint_position()
        print(f"q [rad]: {q}")
        print(f"q [deg]: {np.rad2deg(q)}")
        time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()


if __name__ == "__main__":
    main()
