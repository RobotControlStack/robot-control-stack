import logging

import gymnasium as gym
from rcs._core import common
from rcs._core.common import BaseCameraConfig, RobotPlatform
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
# from rcs_fr3.creators import HardwareCameraCreatorConfig

# from rcs_robotiq2f85.hw import RobotiQ2F85GripperConfig

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
FR3_IP = "192.168.1.12"


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
        from rcs_fr3.configs import DefaultFR3HardwareEnv

        env_creator = DefaultFR3HardwareEnv()
        env_creator.ip = FR3_IP
        hw_cfg = env_creator.config()
        hw_cfg.robot_cfg.tcp_offset = rcs.GRIPPER_OFFSETS[common.GripperType("Robotiq2F85")]
        hw_cfg.robot_cfg.ignore_realtime = True
        hw_cfg.control_mode = ControlMode.CARTESIAN_TQuat
        ZED_CAMERA_DICT = {
            "wrist": "35115330",
            "side": "14943057",
        }
        hw_cfg.camera_cfgs = {}
        hw_cfg.camera_cfgs["zed"] = HardwareCameraCreatorConfig(
                camera_type_id="zed",
                camera_cfgs={
                    name: BaseCameraConfig(
                        identifier=identifier,
                        resolution_width=1280,
                        resolution_height=720,
                        frame_rate=30,
                    )
                    for name, identifier in ZED_CAMERA_DICT.items()
                },
                kwargs={
                    "enable_depth": False,
                    "enable_imu": False,
                    "include_right": True,
                },
        )

        hw_cfg.max_relative_movement = 0.5
        hw_cfg.relative_to = RelativeTo.LAST_STEP
        left_gripper_serial_number = "DAANTG8W"
        hw_cfg.gripper_cfg = RobotiQ2F85GripperConfig(
                serial_number=left_gripper_serial_number,
                speed=100,
                force=50,
                async_control=False,
            )
        env_rel = env_creator.create_env(hw_cfg)
        input("the robot is going to move, press enter whenever you are ready")

    env_rel.reset()

    # access low level robot api to get current cartesian position
    print(env_rel.get_wrapper_attr("robot").get_cartesian_position())  # type: ignore

    for _ in range(100):
        for _ in range(10):
            # move 1cm in x direction (forward) and close gripper
            act = {"tquat": [0.01, 0, 0, 0, 0, 0, 1], "gripper": [0]}
            obs, reward, terminated, truncated, info = env_rel.step(act)
        for _ in range(10):
            # move 1cm in negative x direction (backward) and open gripper
            act = {"tquat": [-0.01, 0, 0, 0, 0, 0, 1], "gripper": [1]}
            obs, reward, terminated, truncated, info = env_rel.step(act)


if __name__ == "__main__":
    main()
