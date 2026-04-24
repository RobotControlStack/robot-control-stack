import logging
from time import sleep

import numpy as np
from rcs._core.common import RobotPlatform
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
from rcs.envs.configs import EmptyWorldUR5e
from rcs.envs.sim import GripperWrapperSim, RobotSimWrapper
from rcs_ur5e.configs import DefaultUR5eHardwareEnv

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

ROBOT_IP = "192.168.25.201"
ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        env_creator = DefaultUR5eHardwareEnv()
        env_creator.ip = ROBOT_IP
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.JOINTS
        hw_cfg.camera_cfgs = None
        hw_cfg.max_relative_movement = np.deg2rad(5)
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        env_rel = env_creator.create_env(hw_cfg)
    else:
        scene = EmptyWorldUR5e()
        sim_cfg_data = scene.prefixed_cfg(scene.config())
        ur5e = scene.lead_robot_name(sim_cfg_data)

        robot_cfg = sim_cfg_data.robot_cfgs[ur5e]
        gripper_cfg = sim_cfg_data.gripper_cfgs[ur5e]  # type: ignore[index]
        camera_cfgs = sim_cfg_data.camera_cfgs
        sim_cfg = SimConfig(
            realtime=False,
            async_control=False,
        )

        kinematic_model_path, attachment_site = scene.kinematics_cfg(sim_cfg_data)[ur5e]
        ik = rcs.common.Pin(
            kinematic_model_path,
            attachment_site,
        )
        mjmodel = scene.create_model(sim_cfg_data)
        simulation = sim.Sim(mjmodel, sim_cfg)

        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env_rel = SimEnv(simulation)
        env_rel = RobotWrapper(env_rel, robot, ControlMode.JOINTS)

        gripper = sim.SimGripper(simulation, gripper_cfg)
        env_rel = GripperWrapper(env_rel, gripper)

        env_rel = RobotSimWrapper(env_rel)
        env_rel = GripperWrapperSim(env_rel)

        if camera_cfgs is not None:
            camera_set = SimCameraSet(simulation, camera_cfgs, physical_units=True, render_on_demand=True)
            env_rel = CameraSetWrapper(env_rel, camera_set, include_depth=True)  # type: ignore[arg-type]

        env_rel = RelativeActionSpace(
            env_rel,
            max_mov=np.deg2rad(5),
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel = CoverWrapper(env_rel)
        env_rel.get_wrapper_attr("sim").open_gui()

    for _ in range(100):
        obs, info = env_rel.reset()
        for _ in range(3):
            # sample random relative action and execute it
            act = env_rel.action_space.sample()
            obs, reward, terminated, truncated, info = env_rel.step(act)
            if truncated or terminated:
                logger.info("Truncated or terminated!")
                return
            sleep(0.4)


if __name__ == "__main__":
    main()
