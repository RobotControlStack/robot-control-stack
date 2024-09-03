import logging

import gymnasium as gym
import rcsss
from rcsss._core.common import Pose
from rcsss._core.sim import FR3, CameraType, FR3Config, SimCameraSet, SimCameraSetConfig
from rcsss.camera.realsense import RealSenseCameraSet
from rcsss.camera.sim import SimCameraConfig
from rcsss.config import read_config_yaml
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    RelativeActionSpace,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim
from rcsss.sim import Sim

logger = logging.getLogger(__name__)


def sim() -> gym.Env:
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
    simulation = Sim(rcsss.scenes["fr3_empty_world"])
    robot = FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    fr3_config = FR3Config()
    fr3_config.realtime = False
    env_sim = FR3Sim(FR3Env(robot, ControlMode.CARTESIAN_TQuart), simulation)
    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=640, resolution_height=480, frame_rate=10)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    gripper_cfg = rcsss.sim.FHConfig()
    gripper = rcsss.sim.FrankaHand(simulation, "0", gripper_cfg)
    env_cam = GripperWrapper(env_cam, gripper)

    return RelativeActionSpace(env_cam)


def hw(roboter_ip) -> gym.Env:
    cfg = read_config_yaml("config.yaml")

    robot = rcsss.hw.FR3(roboter_ip, str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    rcfg = rcsss.hw.FR3Config()
    rcfg.tcp_offset = Pose(rcsss.common.FrankaHandTCPOffset())
    rcfg.speed_factor = 0.2
    rcfg.controller = rcsss.hw.IKController.robotics_library
    robot.set_parameters(rcfg)

    env = FR3Env(robot, ControlMode.CARTESIAN_TRPY)
    env_hw: gym.Env = FR3HW(env)
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.5
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    gripper = rcsss.hw.FrankaHand(roboter_ip, gripper_cfg)
    env_hw = GripperWrapper(env_hw, gripper, binary=True)

    assert cfg.hw.camera_type == "realsense"
    # print(cam_cfg)
    # print(cfg.hw.camera_config.realsense_config)
    camera_set = RealSenseCameraSet(cfg.hw.camera_config.realsense_config)
    camera_set.start()

    env_hw = CameraSetWrapper(env_hw, camera_set)

    env_hw = CollisionGuard.env_from_xml_paths(
        env_hw,
        str(rcsss.scenes["fr3_empty_world"]),
        str(rcsss.scenes["lab"].parent / "fr3.urdf"),
        gripper=True,
        check_home_collision=False,
        camera=True,
        control_mode=ControlMode.CARTESIAN_TRPY,
        tcp_offset=Pose(rcsss.common.FrankaHandTCPOffset()),
    )

    return RelativeActionSpace(env_hw)
