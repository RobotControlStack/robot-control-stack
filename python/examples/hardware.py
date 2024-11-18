import logging
import sys
from time import sleep

import cv2
import numpy as np
import pyrealsense2 as rs
import rcsss
from dotenv import dotenv_values
from rcsss import sim
from rcsss._core.hw import FR3Config, IKController
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.control.fr3_desk import FCI, Desk, DummyResourceManager
from rcsss.control.utils import load_creds_fr3_desk
from rcsss.envs.base import RobotInstance
from rcsss.envs.factories import get_urdf_path, default_realsense
from utils.threed_rendering import reconstruct_3d_hardware

ROBOT_IP = "192.168.101.1"
# ROBOT_INSTANCE = RobotInstance.SIMULATION
ROBOT_INSTANCE = RobotInstance.HARDWARE
# replace this with a path to a robot urdf file if you dont have the utn models
URDF_PATH = None

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

When you use a real FR3 you first need to unlock its joints using the following cli script:

python -m rcsss fr3 unlock <ip>

or put it into guiding mode using:

python -m rcsss fr3 guiding-mode <ip>

When you are done you lock it again using:

python -m rcsss fr3 lock <ip>

or even shut it down using:

python -m rcsss fr3 shutdown <ip>
"""


def main():
    # cameras = {"wrist":"244222071045", "bird-eye":"243522070364"}
    # camera_set = default_realsense({"wrist":"244222071045",
    #                                 "bird-eye":"243522070364"})
    # camera_set.start()
    # print(camera_set.camera_names)
    # print(camera_set.get_depth_shape())
    # camera_set.wait_for_frames()
    # frameset=camera_set.get_latest_frames()
    # print(frameset.frames["wrist"].camera.depth.data)
    # depth = frameset.frames["wrist"].camera.depth.data
    # color = frameset.frames["wrist"].camera.color.data
    # # depth and color of the bird-eye camera
    # depth_be = frameset.frames["bird-eye"].camera.depth.data
    # color_be = frameset.frames["bird-eye"].camera.color.data
    # camera_set.get_device_intrinsics(cameras)
    # reconstruct_3d_hardware(cameras,camera_set, logger, display=True)
    # camera_set.stop()
    # # get depth statistics
    # print(f"depth min: {np.min(depth)}")
    # print(f"depth max: {np.max(depth)}")
    # print(f"depth mean: {np.mean(depth)}")
    # print(f"depth std: {np.std(depth)}")
    # # get be depth statistics
    # print(f"depth min: {np.min(depth_be)}")
    # print(f"depth max: {np.max(depth_be)}")
    # print(f"depth mean: {np.mean(depth_be)}")
    # print(f"depth std: {np.std(depth_be)}")
    # # show the depth image with option to hover for value
    # cv2.imshow("depth", depth)
    # cv2.imshow("color", color)
    # cv2.imshow("depth_be", depth_be)
    # cv2.imshow("color_be", color_be)
    # # # hover for depth value
    # # def on_mouse(event, x, y, flags, param):
    # #     if event == cv2.EVENT_MOUSEMOVE:
    # #         print(f"depth value at {x}, {y}: {depth[y, x]}")
    # #         print(f"depth value at be {x}, {y}: {depth_be[y, x]}")
    # # cv2.setMouseCallback("depth", on_mouse)
    # # cv2.setMouseCallback("depth_be", on_mouse)
    # cv2.waitKey()
    # camera_set.stop()

    # sys.exit()
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        print(user, pw)
        resource_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=True, lock_when_done=False)
    else:
        resource_manger = DummyResourceManager()

    with resource_manger:
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation = sim.Sim(rcsss.scenes["fr3_empty_world"])
            urdf_path = get_urdf_path(URDF_PATH, allow_none_if_not_found=False)
            ik = rcsss.common.IK(urdf_path)
            robot = rcsss.sim.FR3(simulation, "0", ik)
            cfg = sim.FR3Config()
            cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            cfg.realtime = False
            robot.set_parameters(cfg)

            gripper_cfg = sim.FHConfig()
            gripper = sim.FrankaHand(simulation, "0", gripper_cfg)

            # add camera to have a rendering gui
            cameras = {
                "default_free": SimCameraConfig(
                    identifier="", type=int(CameraType.default_free)
                ),
                "wrist": SimCameraConfig(
                    identifier="eye-in-hand_0", type=int(CameraType.fixed)
                ),
                # TODO: odd behavior when not both cameras are used: only last image is shown
            }
            cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=1280, resolution_height=720, frame_rate=20)
            camera_set = SimCameraSet(simulation, cam_cfg)
            simulation.open_gui()

        else:
            urdf_path = get_urdf_path(URDF_PATH, allow_none_if_not_found=False)
            ik = rcsss.common.IK(urdf_path)
            robot = rcsss.hw.FR3(ROBOT_IP, ik)
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            # robot_cfg.controller = IKController.robotics_library
            robot.set_parameters(robot_cfg)
            gripper_cfg = rcsss.hw.FHConfig()
            gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
            gripper_cfg.speed = 0.1
            gripper_cfg.force = 30
            gripper = rcsss.hw.FrankaHand(ROBOT_IP, gripper_cfg)
            input("the robot is going to move, press enter whenever you are ready")

        current_translation = robot.get_cartesian_position().translation()
        current_pose = robot.get_cartesian_position()
        
        # # # move to home position and open gripper
        # robot.move_home()
        # gripper.grasp()
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        # logger.info("Robot is in home position, gripper is open")

        # # 5cm in x direction
        # for i in range(5):
        # print(robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, current_translation[2]-0.15])))
        current_translation = robot.get_cartesian_position().translation()

        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, current_translation[2]-0.025]))
        )
        print(robot.get_cartesian_position(),"current pose")
        # print(robot.get_cartesian_position(),"current pose") # robot to ee

        #     # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     #     simulation.step_until_convergence()
        #     #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     #     logger.debug(f"sim converged: {simulation.is_converged()}")

        #     # # 5cm in y direction
        #     robot.set_cartesian_position(
        #         robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0.07, 0]))
        #     )
        #     # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     #     simulation.step_until_convergence()
        #     #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     #     logger.debug(f"sim converged: {simulation.is_converged()}")

        #     # # 5cm in z direction
        #     robot.set_cartesian_position(
        #         robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, 0.07]))
        #     )
        #     robot.set_cartesian_position(
        #         robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0,-0.07]))
        #     )
        #     robot.set_cartesian_position(
        #         robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0,-0.07, 0]))
        #     )
        #     robot.set_cartesian_position(
        #         robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([-0.07, 0, 0]))
        #     )

        
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     logger.debug(f"sim converged: {simulation.is_converged()}")

        # # rotate the arm 90 degrees around the inverted y and z axis
        # new_pose = robot.get_cartesian_position() * rcsss.common.Pose(
        #     translation=np.array([0, 0, 0]), rpy=rcsss.common.RPY(roll=0, pitch=-np.deg2rad(90), yaw=-np.deg2rad(90))
        # )
        # robot.set_cartesian_position(new_pose)
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     logger.debug(f"sim converged: {simulation.is_converged()}")

        # if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        #     input(
        #         "hold an object 25cm in front of the gripper, the robot is going to grasp it, press enter when you are ready"
        #     )

        # # move 25cm towards the gripper direction
        # robot.set_cartesian_position(
        #     robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, 0.25]))
        # )
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     logger.debug(f"sim converged: {simulation.is_converged()}")

        # # grasp the object
        # gripper.grasp()
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        #     logger.debug(f"sim converged: {simulation.is_converged()}")

        # # move 25cm backward
        # robot.set_cartesian_position(
        #     robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, -0.25]))
        # )
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()
        #     logger.debug(f"IK success: {robot.get_state().ik_success}")
        #     logger.debug(f"sim converged: {simulation.is_converged()}")

        # if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        #     input("gripper is going to be open, press enter when you are ready")

        # # open gripper
        # gripper.open()
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()

        # # move back to home position
        # robot.move_home()
        # if ROBOT_INSTANCE == RobotInstance.SIMULATION:
        #     simulation.step_until_convergence()


if __name__ == "__main__":
    main()
