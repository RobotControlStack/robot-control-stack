import logging

import numpy as np
from rcs._core.common import RobotPlatform
from rcs._core.sim import CameraType, SimConfig
from rcs.camera.sim import SimCameraConfig, SimCameraSet
from rcs.envs.configs import EmptyWorldFR3
from rcs_fr3._core import hw
from rcs_fr3.configs import DefaultFR3HardwareEnv
from rcs_fr3.desk import FCI, ContextManager, Desk, load_creds_franka_desk

import rcs
from rcs import sim

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
DESK_USERNAME=<username on franka desk>
DESK_PASSWORD=<password on franka desk>

When you use a real FR3 you first need to unlock its joints using the following cli script:

python -m rcs_fr3 unlock <ip>

or put it into guiding mode using:

python -m rcs_fr3 guiding-mode <ip>

When you are done you lock it again using:

python -m rcs_fr3 lock <ip>

or even shut it down using:

python -m rcs_fr3 shutdown <ip>
"""


def main():
    context_manger: FCI | ContextManager
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        user, pw = load_creds_franka_desk()
        context_manger = FCI(Desk(ROBOT_IP, user, pw), unlock=False, lock_when_done=False)
    else:
        context_manger = ContextManager()

    with context_manger:
        robot: rcs.common.Robot
        gripper: rcs.common.Gripper
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            scene = EmptyWorldFR3()
            cfg = scene.prefixed_cfg(scene.config())
            fr3 = scene.lead_robot_name(cfg)
            mjmodel = scene.create_model(cfg)

            sim_cfg = SimConfig(
                realtime=False,
                async_control=False,
            )

            simulation = sim.Sim(mjmodel, sim_cfg)

            robot_cfg = cfg.robot_cfgs[fr3]

            kinematic_model_path, attachment_site = scene.kinematics_cfg(cfg)[fr3]
            ik = rcs.common.Pin(
                kinematic_model_path,
                attachment_site,
            )
            robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)

            gripper_cfg = cfg.gripper_cfgs[fr3]  # type: ignore
            gripper = sim.SimGripper(simulation, gripper_cfg)

            # add camera to have a rendering gui
            cameras = {
                "default_free": sim.SimCameraConfig(
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
            default_env = DefaultFR3HardwareEnv()
            default_env.ip = ROBOT_IP
            env_cfg = default_env.config()
            fr3_cfg = env_cfg.robot_cfg
            fr3_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
            ik = rcs.common.Pin(
                fr3_cfg.kinematic_model_path,
                fr3_cfg.attachment_site,
                urdf=fr3_cfg.kinematic_model_path.endswith(".urdf"),
            )
            robot = hw.Franka(fr3_cfg, ik)

            gripper_cfg_hw = env_cfg.gripper_cfg
            assert isinstance(gripper_cfg_hw, hw.FHConfig)
            gripper = hw.FrankaHand(gripper_cfg_hw)
            input("the robot is going to move, press enter whenever you are ready")

        # move to home position and open gripper
        robot.move_home()
        gripper.open()
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
        logger.info("Robot is in home position, gripper is open")

        # 5cm in x direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0.05, 0, 0]))  # type: ignore
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in y direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0.05, 0]))  # type: ignore
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # 5cm in z direction
        robot.set_cartesian_position(
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, 0.05]))  # type: ignore
        )
        if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
            simulation.step_until_convergence()
            logger.debug(f"IK success: {robot.get_state().ik_success}")  # type: ignore
            logger.debug(f"sim converged: {simulation.is_converged()}")

        # rotate the arm 90 degrees around the inverted y and z axis
        new_pose = robot.get_cartesian_position() * rcs.common.Pose(
            translation=np.array([0, 0, 0]), rpy=rcs.common.RPY(roll=0, pitch=-np.deg2rad(90), yaw=-np.deg2rad(90))  # type: ignore
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
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, 0.25]))  # type: ignore
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
            robot.get_cartesian_position() * rcs.common.Pose(translation=np.array([0, 0, -0.25]))  # type: ignore
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
