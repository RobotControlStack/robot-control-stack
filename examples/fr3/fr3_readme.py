from time import sleep

import gymnasium as gym
import numpy as np
from rcs._core.sim import SimConfig
from rcs.camera.sim import SimCameraSet
from rcs.envs.base import (
    CameraSetWrapper,
    ControlMode,
    CoverWrapper,
    GripperWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldFR3
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper

import rcs
from rcs import sim

if __name__ == "__main__":
    # default configs
    scene = EmptyWorldFR3()
    cfg = scene.prefixed_cfg(scene.config())
    fr3 = scene.lead_robot_name(cfg)

    robot_cfg = cfg.robot_cfgs[fr3]
    gripper_cfg = cfg.gripper_cfgs[fr3]  # type: ignore
    camera_cfgs = cfg.camera_cfgs
    sim_cfg = SimConfig(
        realtime=True,
        async_control=True,
        frequency=1,  # in Hz (1 sec delay)
    )
    mjmodel = scene.create_model(cfg)
    kinematic_model_path, attachment_site = scene.kinematics_cfg(cfg)[fr3]

    simulation = sim.Sim(mjmodel, sim_cfg)
    ik = rcs.common.Pin(
        kinematic_model_path,
        attachment_site,
    )

    # base env
    robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
    env: gym.Env = SimEnv(simulation)
    env = RobotWrapper(env, robot, ControlMode.CARTESIAN_TQuat)

    # gripper
    gripper = sim.SimGripper(simulation, gripper_cfg)
    env = GripperWrapper(env, gripper)

    env = RobotSimWrapper(env)
    env = GripperWrapperSim(env)

    # camera
    camera_set = SimCameraSet(simulation, camera_cfgs, physical_units=True, render_on_demand=True)  # type: ignore
    env = CameraSetWrapper(env, camera_set, include_depth=True)  # type: ignore

    # relative actions bounded by 10cm translation and 10 degree rotation
    env = RelativeActionSpace(env, max_mov=(0.1, np.deg2rad(10)), relative_to=RelativeTo.LAST_STEP)
    env = CoverWrapper(env)

    env.get_wrapper_attr("sim").open_gui()
    # wait for gui to open
    sleep(1)
    env.reset()

    # access low level robot api to get current cartesian position
    print(env.get_wrapper_attr("robot").get_cartesian_position())

    for _ in range(10):
        # move 1cm in x direction (forward) and close gripper
        act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}
        obs, reward, terminated, truncated, info = env.step(act)
        print(obs)
