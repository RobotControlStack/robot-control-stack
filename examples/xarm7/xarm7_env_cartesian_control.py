import logging
from time import sleep

from rcs._core.common import RobotPlatform
from rcs._core.sim import SimConfig
from rcs.envs.base import (
    ControlMode,
    CoverWrapper,
    RelativeActionSpace,
    RelativeTo,
    RobotWrapper,
    SimEnv,
)
from rcs.envs.configs import EmptyWorldXArm7
from rcs.envs.sim import RobotSimWrapper
from rcs_xarm7.configs import DefaultXArm7HardwareEnv

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

"""
The example shows how to create a xArm7 environment with Cartesian control
and a relative action space. The example works both with a real robot and in
simulation.

To test with a real robot, set ROBOT_INSTANCE to RobotPlatform.HARDWARE,
install the rcs_xarm7 extension (`pip install extensions/rcs_xarm7`)
and set the ROBOT_IP variable to the robot's IP address.
"""

ROBOT_IP = "192.168.1.245"
ROBOT_INSTANCE = RobotPlatform.SIMULATION
# ROBOT_INSTANCE = RobotPlatform.HARDWARE


def main():

    if ROBOT_INSTANCE == RobotPlatform.HARDWARE:
        env_creator = DefaultXArm7HardwareEnv()
        env_creator.ip = ROBOT_IP
        hw_cfg = env_creator.config()
        hw_cfg.control_mode = ControlMode.CARTESIAN_TQuat
        hw_cfg.max_relative_movement = 0.5
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        env_rel = env_creator.create_env(hw_cfg)
    else:
        scene = EmptyWorldXArm7()
        sim_cfg_data = scene.prefixed_cfg(scene.config())
        xarm7 = scene.lead_robot_name(sim_cfg_data)

        robot_cfg = sim_cfg_data.robot_cfgs[xarm7]
        sim_cfg = SimConfig(
            realtime=False,
            async_control=False,
        )

        mjmodel = scene.create_model(sim_cfg_data)
        simulation = sim.Sim(mjmodel, sim_cfg)

        kinematic_model_path, attachment_site = scene.kinematics_cfg(sim_cfg_data)[xarm7]
        ik = rcs.common.Pin(
            kinematic_model_path,
            attachment_site,
        )

        robot = rcs.sim.SimRobot(simulation, ik, robot_cfg)
        env_rel = SimEnv(simulation)
        env_rel = RobotWrapper(env_rel, robot, ControlMode.CARTESIAN_TQuat)
        env_rel = RobotSimWrapper(env_rel)
        env_rel = RelativeActionSpace(
            env_rel,
            max_mov=0.5,
            relative_to=RelativeTo.LAST_STEP,
        )
        env_rel = CoverWrapper(env_rel)
        env_rel.get_wrapper_attr("sim").open_gui()
        sleep(3)  # wait for gui to open
    obs, info = env_rel.reset()

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.6)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
            sleep(0.6)


if __name__ == "__main__":
    main()
