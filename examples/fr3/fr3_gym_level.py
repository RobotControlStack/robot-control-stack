import logging

import gymnasium as gym
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    HardwareEnv,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldFR3
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
This script demonstrates how to control the FR3 robot in Cartesian position control mode
using relative movements. The robot (or its simulation) moves 1cm forward and then 1cm backward
in a loop while opening and closing the gripper.

To control a real FR3 robot, install the rcs_fr3 extension (`pip install extensions/rcs_fr3`),
and set the FR3_IP variable to the robot's IP address. Make sure to unlock the robot's joints and
put it into FCI mode before running this script. For a scripted way of unlocking and guiding mode see the
fr3_direct_control.py example which uses the FCI context manager.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION
FR3_IP = "192.168.101.1"
USE_FRANKA_HAND = True


def main():
    if ROBOT_INSTANCE == RobotPlatform.SIMULATION:
        scene = EmptyWorldFR3()
        cfg = scene.prefixed_cfg(scene.config())
        fr3 = scene.lead_robot_name(cfg)

        robot_cfg = cfg.robot_cfgs[fr3]
        gripper_cfg = cfg.gripper_cfgs[fr3]  # type: ignore
        camera_cfgs = cfg.camera_cfgs
        sim_cfg = SimConfig(
            realtime=False,
            async_control=False,
        )

        mjmodel = scene.create_model(cfg)
        simulation = sim.Sim(mjmodel, sim_cfg)

        kinematic_model_path, attachment_site = scene.kinematics_cfg(cfg)[fr3]
        ik = rcs.common.Pin(
            kinematic_model_path,
            attachment_site,
        )

        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env_rel: gym.Env = SimEnv(simulation)
        env_rel = RobotWrapper(env_rel, robot, ControlMode.CARTESIAN_TQuat)

        gripper = sim.SimGripper(simulation, gripper_cfg)
        env_rel = GripperWrapper(env_rel, gripper)

        env_rel = RobotSimWrapper(env_rel)
        env_rel = GripperWrapperSim(env_rel)

        camera_set = SimCameraSet(simulation, camera_cfgs, physical_units=True, render_on_demand=True)  # type: ignore
        env_rel = CameraSetWrapper(env_rel, camera_set, include_depth=True)  # type: ignore[arg-type]

        env_rel = RelativeActionSpace(env_rel, max_mov=0.5, relative_to=RelativeTo.LAST_STEP)
        env_rel = CoverWrapper(env_rel)
        env_rel.get_wrapper_attr("sim").open_gui()
    else:
        from rcs_fr3._core import hw
        from rcs_fr3.configs import DefaultSingleFR3HardwareEnv
        from rcs_fr3.envs import FR3HW

        default_env = DefaultSingleFR3HardwareEnv()
        default_env.ip = FR3_IP
        hw_cfg = default_env.config()

        robot_cfg = hw_cfg.robot_cfg
        ik = rcs.common.Pin(
            robot_cfg.kinematic_model_path,
            robot_cfg.attachment_site,
            urdf=robot_cfg.kinematic_model_path.endswith(".urdf"),
        )
        robot = hw.Franka(robot_cfg, ik)

        env_rel = HardwareEnv()
        env_rel = RobotWrapper(
            env_rel,
            robot,
            ControlMode.CARTESIAN_TQuat,
            home_on_reset=hw_cfg.wrapper_cfg.home_on_reset,
        )
        env_rel = FR3HW(env_rel)

        gripper_cfg = hw_cfg.gripper_cfg
        if USE_FRANKA_HAND:
            assert isinstance(gripper_cfg, hw.FHConfig)
            gripper = hw.FrankaHand(gripper_cfg)
            env_rel = GripperWrapper(env_rel, gripper, binary=hw_cfg.wrapper_cfg.binary_gripper)

        env_rel = RelativeActionSpace(env_rel, max_mov=0.5, relative_to=RelativeTo.LAST_STEP)
        env_rel = CoverWrapper(env_rel)
        input("the robot is going to move, press enter whenever you are ready")

    env_rel.reset()

    # access low level robot api to get current cartesian position
    print(env_rel.get_wrapper_attr("robot").get_cartesian_position())  # type: ignore

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}
            # you can also sample an action from the action space
            # act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)

        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)


if __name__ == "__main__":
    main()
