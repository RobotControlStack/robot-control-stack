"""Script for testing UR5e robot connection and basic movements."""

import time

from rcs_ur5e.hw import UR5e, UR5eConfig

import rcs

ROBOT_IP = "192.168.25.201"
robot_config = UR5eConfig(ip=ROBOT_IP)
ik = rcs.common.Pin(
    robot_config.kinematic_model_path,
    robot_config.attachment_site,
    urdf=robot_config.kinematic_model_path.endswith(".urdf"),
)
robot = UR5e(robot_config, ik)
print(f"Robot joint positions: {robot.get_joint_position()}")
print(f"Robot cartesian position: {robot.get_cartesian_position()}")
print(f"Robot Config: {robot.get_config()}")

input("Press Enter to continue and move to home position...")
robot.move_home()
input("Press Enter to continue...")
target_q = robot.get_joint_position()
target_q[0] += 0.05  # Move slightly
print(f"Setting joint position to {target_q}")
for _ in range(100):
    target_q[0] += 0.005  # Move slightly
    robot.set_joint_position(target_q)
    time.sleep(0.05)
