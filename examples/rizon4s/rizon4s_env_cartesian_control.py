import logging
from time import sleep

import gymnasium as gym
import numpy as np
import rcs
from rcs import sim
from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import (
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldRizon4S
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
This script demonstrates Cartesian position control of the Flexiv Rizon 4s arm in synchronous mode.
The arm is driven to its home pose on reset and then moves 1cm forward and backward along the base x
axis in a loop while opening and closing the Flexiv Grav gripper. Every step goes through inverse
kinematics, so the printed TCP positions tracking the commanded ones show that IK works.

To control a real Rizon 4s, install the rcs_flexiv extension (`pip install -ve extensions/rcs_flexiv`),
set ROBOT_SN and the gripper's tool name and set ROBOT_INSTANCE to RobotPlatform.HARDWARE. The robot
has to be in auto mode with the E-stop released, see the extension README.
"""

ROBOT_INSTANCE = RobotPlatform.HARDWARE  # Change to RobotPlatform.HARDWARE for the real arm
ROBOT_SN = "Rizon4s-063650"
GRIPPER_TOOL_NAME = None  # tool created for the Grav in Flexiv Elements, None keeps the active tool

STEP_SIZE = 0.01  # meters per step
STEPS_PER_LEG = 10  # steps forward before reversing
CYCLES = 100


def main():
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_flexiv.configs import DefaultRizon4SHardwareEnv

        env_creator = DefaultRizon4SHardwareEnv()
        env_creator.robot_sn = ROBOT_SN
        env_creator.tool_name = GRIPPER_TOOL_NAME
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.CARTESIAN_TQuat
        # Synchronous mode: every command returns once the arm has reached its target.
        hw_cfg.robot_cfg.async_control = False
        hw_cfg.max_relative_movement = (0.05, np.deg2rad(5))
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        hw_cfg.frequency = 25  # limit the rate at which we send commands to the robot
        env_hw = env_creator.create_env(hw_cfg)
        input("the arm is going to move, press enter whenever you are ready")
        run(env_hw)
        return

    scene = EmptyWorldRizon4S()
    sim_cfg_data = scene.prefixed_cfg(scene.config())
    rizon = scene.lead_robot_name(sim_cfg_data)

    robot_cfg = sim_cfg_data.robot_cfgs[rizon]
    gripper_cfg = sim_cfg_data.gripper_cfgs[rizon]  # type: ignore[index]
    # Synchronous mode: the simulation steps until the commanded pose is reached.
    sim_cfg = SimConfig(
        realtime=False,
        async_control=False,
    )

    mjmodel = scene.create_model(sim_cfg_data)
    simulation = sim.Sim(mjmodel, sim_cfg)

    kinematic_model_path, attachment_site = scene.kinematics_cfg(sim_cfg_data)[rizon]
    ik = rcs.common.Pin(
        kinematic_model_path,
        attachment_site,
    )

    robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
    env_rel = SimEnv(simulation)
    env_rel = RobotWrapper(env_rel, robot, ControlMode.CARTESIAN_TQuat)

    gripper = sim.SimGripper(simulation, gripper_cfg)
    env_rel = GripperWrapper(env_rel, gripper)

    env_rel = RobotSimWrapper(env_rel)
    env_rel = GripperWrapperSim(env_rel)

    env_rel = RelativeActionSpace(
        env_rel,
        max_mov=(0.05, np.deg2rad(5)),
        relative_to=RelativeTo.LAST_STEP,
    )
    env_rel = CoverWrapper(env_rel)
    env_rel.get_wrapper_attr("sim").open_gui()
    run(env_rel)


def run(env_rel: gym.Env) -> None:
    # Homing happens on reset, driving the joints to the home pose.
    env_rel.reset()

    robot_api = env_rel.get_wrapper_attr("robot")
    print(f"home TCP: {np.round(robot_api.get_cartesian_position().translation(), 4)}")

    for _ in range(CYCLES):
        for _ in range(STEPS_PER_LEG):
            # move forward along x and close the gripper
            act = {"tquat": [STEP_SIZE, 0, 0, 0, 0, 0, 1], "gripper": [0]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(f"TCP: {np.round(robot_api.get_cartesian_position().translation(), 4)}")
            sleep(0.1)
        for _ in range(STEPS_PER_LEG):
            # move backward along x and open the gripper
            act = {"tquat": [-STEP_SIZE, 0, 0, 0, 0, 0, 1], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            print(f"TCP: {np.round(robot_api.get_cartesian_position().translation(), 4)}")
            sleep(0.1)


if __name__ == "__main__":
    main()
