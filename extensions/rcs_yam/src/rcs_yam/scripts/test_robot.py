"""Script for testing a YAM arm connection, sync and async joint control and the gripper.

Run with the CAN bus up, see the extension README. Keep the workspace clear, the arm moves.
"""

import time

import numpy as np
from rcs_yam.hw import Yam, YamConfig, YamGripper

import rcs
from rcs import common

CHANNEL = "can0"

robot_type = common.RobotType("Yam")
gripper_type = common.GripperType("Yam")
robot_config = YamConfig(
    channel=CHANNEL,
    async_control=False,
    robot_type=robot_type,
    kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
    attachment_site=rcs.ROBOTS[robot_type].attachment_site,
    dof=rcs.ROBOTS[robot_type].dof,
    joint_limits=rcs.ROBOTS[robot_type].joint_limits,
    q_home=rcs.ROBOTS[robot_type].q_home,
    tcp_offset=rcs.GRIPPER_TCP_OFFSETS[gripper_type],
)
ik = rcs.common.Pin(
    robot_config.kinematic_model_path,
    robot_config.attachment_site,
    urdf=robot_config.kinematic_model_path.endswith(".urdf"),
)
robot = Yam(robot_config, ik)
gripper = YamGripper(common.GripperConfig(gripper_type=gripper_type), robot)

print(f"Joint positions: {robot.get_joint_position()}")
print(f"Cartesian position: {robot.get_cartesian_position()}")
print(f"Gripper width: {gripper.get_normalized_width():.3f}")

input("Press Enter to move to the home position...")
robot.move_home()

input("Press Enter for a small synchronous joint move...")
target_q = robot.get_joint_position()
target_q[0] += 0.2
start = time.time()
robot.set_joint_position(target_q)
print(f"sync command returned after {time.time() - start:.3f} s at {robot.get_joint_position()}")

input("Press Enter for the same move asynchronously...")
cfg = robot.get_config()
cfg.async_control = True
robot.set_config(cfg)
target_q[0] -= 0.2
start = time.time()
robot.set_joint_position(target_q)
print(f"async command returned after {time.time() - start:.3f} s at {robot.get_joint_position()}")
time.sleep(1.0)
print(f"one second later: {robot.get_joint_position()}")

input("Press Enter for a small cartesian move...")
cfg.async_control = False
robot.set_config(cfg)
pose = robot.get_cartesian_position()
robot.set_cartesian_position(
    common.Pose(translation=pose.translation() + np.array([0.0, 0.0, -0.03]), quaternion=pose.rotation_q())
)
print(f"cartesian position now: {robot.get_cartesian_position()}")

input("Press Enter to cycle the gripper...")
for width in (0.0, 1.0):
    gripper.set_normalized_width(width)
    print(f"commanded {width:.1f}, measured {gripper.get_normalized_width():.3f}")

robot.close()
print("done")
