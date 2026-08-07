import logging

import gymnasium as gym
import numpy as np
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
from rcs.envs.configs import EmptyWorldYam
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
This script demonstrates Cartesian position control of the YAM arm in synchronous mode. The arm
first moves to its home pose, ramped rather than snapped, and then moves 1cm forward and backward
along the base x axis in a loop. Every step goes through inverse kinematics, so the printed TCP
positions tracking the commanded ones show that IK works.

To control a real YAM arm, install the rcs_yam extension (`pip install -ve extensions/rcs_yam`),
bring up its CAN interface (`sudo ip link set can0 up type can bitrate 1000000`) and set
ROBOT_INSTANCE to RobotPlatform.HARDWARE. Note that the linear_4310 gripper calibrates on startup
and drives its fingers to both end stops.
"""

ROBOT_INSTANCE = RobotPlatform.SIMULATION  # Change to RobotPlatform.HARDWARE for the real arm
CAN_CHANNEL = "can0"

STEP_SIZE = 0.01  # meters per step
STEPS_PER_LEG = 5  # steps forward before reversing
CYCLES = 3


def main():
    env_rel: gym.Env
    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        from rcs_yam.configs import DefaultYamHardwareEnv

        env_creator = DefaultYamHardwareEnv()
        env_creator.channel = CAN_CHANNEL
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.CARTESIAN_TQuat
        # Synchronous mode: every command returns once the arm has reached its target.
        hw_cfg.robot_cfg.async_control = False
        # Homing interpolates over this duration instead of stepping to the home pose.
        hw_cfg.robot_cfg.move_home_duration = 3.0
        hw_cfg.max_relative_movement = (0.05, np.deg2rad(5))
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        env_rel = env_creator.create_env(hw_cfg)
        input("the arm is going to move, press enter whenever you are ready")
    else:
        scene = EmptyWorldYam()
        sim_cfg_data = scene.prefixed_cfg(scene.config())
        yam = scene.lead_robot_name(sim_cfg_data)

        robot_cfg = sim_cfg_data.robot_cfgs[yam]
        gripper_cfg = sim_cfg_data.gripper_cfgs[yam]  # type: ignore[index]
        # Synchronous mode: the simulation steps until the commanded pose is reached.
        sim_cfg = SimConfig(
            realtime=False,
            async_control=False,
        )

        mjmodel = scene.create_model(sim_cfg_data)
        simulation = sim.Sim(mjmodel, sim_cfg)

        kinematic_model_path, attachment_site = scene.kinematics_cfg(sim_cfg_data)[yam]
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

    # Homing happens on reset, driving the joints to the home pose.
    env_rel.reset()

    robot_api = env_rel.get_wrapper_attr("robot")
    print(f"home TCP: {np.round(robot_api.get_cartesian_position().translation(), 4)}")

    for _ in range(CYCLES):
        for direction in (1.0, -1.0):
            for _ in range(STEPS_PER_LEG):
                before = robot_api.get_cartesian_position().translation()
                # Relative to the current pose: move along the base x axis, keep the orientation.
                act = {"tquat": [direction * STEP_SIZE, 0, 0, 0, 0, 0, 1.0], "gripper": [1]}
                env_rel.step(act)
                after = robot_api.get_cartesian_position().translation()
                print(
                    f"commanded {direction * STEP_SIZE:+.3f} m in x: "
                    f"TCP {np.round(before, 4)} -> {np.round(after, 4)}, "
                    f"tracking error {np.linalg.norm(after - (before + np.array([direction * STEP_SIZE, 0, 0]))):.4f} m"
                )


if __name__ == "__main__":
    main()
