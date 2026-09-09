import logging
from time import sleep

import gymnasium as gym
import numpy as np
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

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
This script demonstrates Cartesian position control of the Flexiv Rizon 4s arm in simulation. The
arm is driven to its home pose on reset and then moves 1cm forward and backward along the base x
axis in a loop while opening and closing the Robotiq 2F85 gripper. Every step goes through inverse
kinematics, so the printed TCP positions tracking the commanded ones show that IK works.

Hardware support for the Rizon 4s is not part of rcs yet, so this example is simulation only.
"""

STEP_SIZE = 0.01  # meters per step
STEPS_PER_LEG = 10  # steps forward before reversing
CYCLES = 100


def main():
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
    env_rel: gym.Env = SimEnv(simulation)
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
