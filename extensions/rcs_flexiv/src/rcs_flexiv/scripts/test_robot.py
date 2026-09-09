"""Script for testing a Flexiv arm connection, sync and async control, stiffness changes and the gripper.

Usage: python -m rcs_flexiv.scripts.test_robot <robot-sn> [<gripper-device-name>]

Keep the workspace clear, the arm and the gripper move. The robot has to be in auto mode with the
E-stop released, see the extension README.
"""

import sys
import time

import numpy as np
import rcs
from rcs import common

from rcs_flexiv.hw import (
    Flexiv,
    FlexivConfig,
    FlexivControlMode,
    FlexivGripper,
    FlexivGripperConfig,
)


def main() -> None:
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    robot_sn = sys.argv[1]
    gripper_device = sys.argv[2] if len(sys.argv) > 2 else None

    robot_type = common.RobotType("Rizon4S")
    gripper_type = common.GripperType("FlexivGrav")
    robot_config = FlexivConfig(
        robot_sn=robot_sn,
        control_mode=FlexivControlMode.JOINT_IMPEDANCE,
        async_control=False,
        robot_type=robot_type,
        kinematic_model_path=rcs.ROBOTS[robot_type].mjcf_model_path,
        attachment_site=rcs.ROBOTS[robot_type].attachment_site,
        dof=rcs.ROBOTS[robot_type].dof,
        joint_limits=rcs.ROBOTS[robot_type].joint_limits,
        q_home=rcs.ROBOTS[robot_type].q_home,
        tcp_offset=rcs.GRIPPER_TCP_OFFSETS[gripper_type],
    )
    ik = rcs.common.Pin(robot_config.kinematic_model_path, robot_config.attachment_site)
    robot = Flexiv(robot_config, ik)

    print(f"Joint positions: {robot.get_joint_position()}")
    print(f"Flange pose (robot): {robot.get_cartesian_flange_position()}")
    print(f"Flange pose (rcs model): {ik.forward(robot.get_joint_position(), common.Pose())}")
    print(f"Nominal joint stiffness: {robot.nominal_joint_stiffness}")
    print(f"Nominal cartesian stiffness: {robot.nominal_cartesian_stiffness}")

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
    time.sleep(1.5)
    print(f"1.5 seconds later: {robot.get_joint_position()}")

    input("Press Enter to halve the joint stiffness (push the arm to feel it), Enter again to restore...")
    robot.set_joint_impedance(0.5 * robot.nominal_joint_stiffness)
    input()
    robot.set_joint_impedance(None)

    input("Press Enter for a small synchronous cartesian move (3 cm down) via the rcs IK...")
    cfg.async_control = False
    robot.set_config(cfg)
    pose = robot.get_cartesian_position()
    robot.set_cartesian_position(
        common.Pose(translation=pose.translation() + np.array([0.0, 0.0, -0.03]), quaternion=pose.rotation_q())
    )
    print(f"cartesian position now: {robot.get_cartesian_position()}")

    input("Press Enter for the same move back up with the robot's cartesian impedance controller...")
    cfg.control_mode = FlexivControlMode.CARTESIAN_IMPEDANCE
    robot.set_config(cfg)
    robot.set_cartesian_position(pose)
    print(f"cartesian position now: {robot.get_cartesian_position()}")
    cfg.control_mode = FlexivControlMode.JOINT_IMPEDANCE
    robot.set_config(cfg)

    if gripper_device is not None:
        input("Press Enter to cycle the gripper...")
        gripper = FlexivGripper(FlexivGripperConfig(device_name=gripper_device, async_control=False), robot)
        for width in (0.0, 1.0):
            gripper.set_normalized_width(width)
            print(f"commanded {width:.1f}, measured {gripper.get_normalized_width():.3f}")
        gripper.close()

    robot.close()
    print("done")


if __name__ == "__main__":
    main()
