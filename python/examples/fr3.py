import logging
import sys

import numpy as np
import rcsss
from dotenv import dotenv_values
from rcsss import sim
from rcsss._core.hw import FR3Config, IKController
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.desk import FCI, Desk, DummyResourceManager
from rcsss.envs.base import RobotInstance

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.SIMULATION

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
logger.addHandler(logging.StreamHandler())

"""
This examples demonstrates the direct robot control api without gym env.

First the robot is moved in x, y and z direction 5cm. Then the arm is rotated to the right
and trying to grasp an object placed 25cm to the right of it. Afterwards it moves back
to the home position.


Create a .env file in the same directory as this file with the following content:
FR3_USERNAME=<username on franka desk>
FR3_PASSWORD=<password on franka desk>
"""


def main():
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        creds = dotenv_values()
        resource_manger = FCI(
            Desk(ROBOT_IP, creds["FR3_USERNAME"], creds["FR3_PASSWORD"]), unlock=False, lock_when_done=False
        )
    else:
        resource_manger = DummyResourceManager()


    with resource_manger:
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation = sim.Sim(rcsss.scenes["fr3_empty_world"])
            robot = rcsss.sim.FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
            cfg = sim.FR3Config()
            cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            cfg.ik_duration_in_milliseconds = 300
            cfg.realtime = False
            robot.set_parameters(cfg)

            gripper_cfg = sim.FHConfig()
            gripper = sim.FrankaHand(simulation, "0", gripper_cfg)

            # add camera to have a rendering gui
            cameras = {
                "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
                "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
                # TODO: odd behavior when not both cameras are used: only last image is shown
            }
            cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=1280, resolution_height=720, frame_rate=20)
            camera_set = SimCameraSet(simulation, cam_cfg)
            resource_manger = DummyResourceManager()

        else:
            robot = rcsss.hw.FR3(ROBOT_IP, str(rcsss.scenes["lab"].parent / "fr3.urdf"))
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            robot_cfg.controller = IKController.robotics_library
            robot.set_parameters(robot_cfg)

            gripper_cfg = rcsss.hw.FHConfig()
            gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
            gripper_cfg.speed = 0.1
            gripper_cfg.force = 30
            gripper = rcsss.hw.FrankaHand(ROBOT_IP, gripper_cfg)
            creds = dotenv_values()
            resource_manger = FCI(
                Desk(ROBOT_IP, creds["FR3_USERNAME"], creds["FR3_PASSWORD"]), unlock=False, lock_when_done=False
            )
            input("the robot is going to move, press enter whenever you are ready")


        # move to home position and open gripper
        robot.move_home()
        gripper.open()
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
        logger.info("Robot is in home position, gripper is open")

        # 5cm in x direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0.05, 0, 0]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in y direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0.05, 0]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in z direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, 0.05]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # rotate the arm 90 degrees around the inverted y and z axis
        new_pose = robot.get_cartesian_position() * rcsss.common.Pose(
            translation=np.array([0, 0, 0]), rpy=rcsss.common.RPY(roll=0, pitch=-np.deg2rad(90), yaw=-np.deg2rad(90))
        )
        robot.set_cartesian_position(new_pose)
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            input(
                "hold an object 25cm in front of the gripper, the robot is going to grasp it, press enter when you are ready"
            )

        # move 25cm towards the gripper direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, 0.25]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # grasp the object
        gripper.grasp()
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # move 25cm backward
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, -0.25]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")
            logger.debug(f"sim converged: {simulation.is_converged()}")

        if ROBOT_INSTANCE == RobotInstance.HARDWARE:
            input("gripper is going to be open, press enter when you are ready")

        # open gripper
        gripper.open()
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()

        # move back to home position
        robot.move_home()
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()


if __name__ == "__main__":
    main()
