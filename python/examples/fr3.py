import logging
import sys

import numpy as np
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config, IKSolver
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.control.fr3_desk import FCI, ContextManager, Desk, load_creds_fr3_desk
from rcsss.envs.base import RobotInstance
from rcsss.envs.creators import get_urdf_path

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotInstance.SIMULATION
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
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    context_manger: FCI | ContextManager
    if ROBOT_INSTANCE == RobotInstance.HARDWARE:
        user, pw = load_creds_fr3_desk()
        context_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        context_manger = ContextManager()

    with context_manger:
        robot: rcsss.common.Robot
        gripper: rcsss.common.Gripper
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation = sim.Sim(rcsss.scenes["fr3_empty_world"])
            urdf_path = get_urdf_path(URDF_PATH, allow_none_if_not_found=False)
            assert urdf_path is not None
            ik = rcsss.common.IK(urdf_path)
            robot = rcsss.sim.FR3(simulation, "0", ik)
            cfg = sim.FR3Config()
            cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            cfg.realtime = False
            robot.set_parameters(cfg)

            gripper_cfg_sim = sim.FHConfig()
            gripper = sim.FrankaHand(simulation, "0", gripper_cfg_sim)

            # add camera to have a rendering gui
            cameras = {
                "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free)),
                "wrist": SimCameraConfig(identifier="wrist_0", type=int(CameraType.fixed)),
            }
            cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=1280, resolution_height=720, frame_rate=20)
            camera_set = SimCameraSet(simulation, cam_cfg)  # noqa: F841
            simulation.open_gui()

        else:
            urdf_path = get_urdf_path(URDF_PATH, allow_none_if_not_found=False)
            assert urdf_path is not None
            ik = rcsss.common.IK(urdf_path)
            robot = rcsss.hw.FR3(ROBOT_IP, ik)
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
            robot_cfg.ik_solver = IKSolver.rcs
            robot.set_parameters(robot_cfg)

            gripper_cfg_hw = rcsss.hw.FHConfig()
            gripper_cfg_hw.epsilon_inner = gripper_cfg_hw.epsilon_outer = 0.1
            gripper_cfg_hw.speed = 0.1
            gripper_cfg_hw.force = 30
            gripper = rcsss.hw.FrankaHand(ROBOT_IP, gripper_cfg_hw)
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
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in y direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0.05, 0]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in z direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcsss.common.Pose(translation=np.array([0, 0, 0.05]))
        )
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # rotate the arm 90 degrees around the inverted y and z axis
        new_pose = robot.get_cartesian_position() * rcsss.common.Pose(
            translation=np.array([0, 0, 0]), rpy=rcsss.common.RPY(roll=0, pitch=-np.deg2rad(90), yaw=-np.deg2rad(90))
        )
        robot.set_cartesian_position(new_pose)
        if ROBOT_INSTANCE == RobotInstance.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
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
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
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
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
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
