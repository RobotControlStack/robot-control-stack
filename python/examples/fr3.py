import logging

import numpy as np
import rcs
from rcs import sim
from rcs._core.common import RobotPlatform
from rcs._core.hw import FR3Config, IKSolver
from rcs._core.sim import CameraType
from rcs.camera.sim import SimCameraConfig, SimCameraSet
from rcs.control.fr3_desk import FCI, ContextManager, Desk, load_creds_fr3_desk

ROBOT_IP = "192.168.101.1"
ROBOT_INSTANCE = RobotPlatform.SIMULATION

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

python -m rcs fr3 unlock <ip>

or put it into guiding mode using:

python -m rcs fr3 guiding-mode <ip>

When you are done you lock it again using:

python -m rcs fr3 lock <ip>

or even shut it down using:

python -m rcs fr3 shutdown <ip>
"""


def main():
    context_manger: FCI | ContextManager
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        user, pw = load_creds_fr3_desk()
        context_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        context_manger = ContextManager()

    with context_manger:
        robot: rcs.common.Robot
        gripper: rcs.common.Gripper
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation = sim.Sim(rcs.scenes["fr3_empty_world"]["mjb"])
            urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
            ik = rcs.common.IK(str(urdf_path))
            cfg = sim.SimRobotConfig()
            cfg.add_id("0")
            cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
            robot = rcs.sim.SimRobot(simulation, ik, cfg)

            gripper_cfg_sim = sim.SimGripperConfig()
            gripper_cfg_sim.add_id("0")
            gripper = sim.SimGripper(simulation, gripper_cfg_sim)

            # add camera to have a rendering gui
            cameras = {
                "default_free": SimCameraConfig(
                    identifier="",
                    type=CameraType.default_free,
                    resolution_width=1280,
                    resolution_height=720,
                    frame_rate=20,
                ),
                "wrist": SimCameraConfig(
                    identifier="wrist_0",
                    type=CameraType.fixed,
                    resolution_width=640,
                    resolution_height=480,
                    frame_rate=30,
                ),
            }
            camera_set = SimCameraSet(simulation, cameras)  # noqa: F841
            simulation.open_gui()

        else:
            urdf_path = rcs.scenes["fr3_empty_world"]["urdf"]
            ik = rcs.common.IK(str(urdf_path))
            robot = rcs.hw.FR3(ROBOT_IP, ik)
            robot_cfg = FR3Config()
            robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
            robot_cfg.ik_solver = IKSolver.rcs_ik
            robot.set_parameters(robot_cfg)

            gripper_cfg_hw = rcs.hw.FHConfig()
            gripper_cfg_hw.epsilon_inner = gripper_cfg_hw.epsilon_outer = 0.1
            gripper_cfg_hw.speed = 0.1
            gripper_cfg_hw.force = 30
            gripper = rcs.hw.FrankaHand(ROBOT_IP, gripper_cfg_hw)
            input("the robot is going to move, press enter whenever you are ready")

        # move to home position and open gripper
        robot.move_home()
        gripper.open()
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
        logger.info("Robot is in home position, gripper is open")

        # 5cm in x direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.05, 0, 0]))
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in y direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0.05, 0]))
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in z direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, 0.05]))
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # rotate the arm 90 degrees around the inverted y and z axis
        new_pose = robot.get_cartesian_position() * rcs.common.Pose(
            translation=np.array([0, 0, 0]), rpy=rcs.common.RPY(roll=0, pitch=-np.deg2rad(90), yaw=-np.deg2rad(90))
        )
        robot.set_cartesian_position(new_pose)
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
            input(
                "hold an object 25cm in front of the gripper, the robot is going to grasp it, press enter when you are ready"
            )

        # move 25cm towards the gripper direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, 0.25]))
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # grasp the object
        gripper.grasp()
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # move 25cm backward
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, -0.25]))
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
            input("gripper is going to be open, press enter when you are ready")

        # open gripper
        gripper.open()
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()

        # move back to home position
        robot.move_home()
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()


if __name__ == "__main__":
    main()
