import logging
import sys

import gymnasium as gym
import numpy as np
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config
from rcsss._core.sim import CameraType
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    LimitedJointsRelDictType,
    ObsArmsGr,
    ObsArmsGrCam,
    RelativeActionSpace,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def hw_env_rel(ip: str, control_mode: ControlMode) -> gym.Env[ObsArmsGr, LimitedJointsRelDictType]:
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    robot = rcsss.hw.FR3(ip, str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    robot_cfg = FR3Config()
    robot.set_parameters(robot_cfg)
    # the robot itself is always controlled in joint space with the robotics library IK
    env = FR3Env(robot, ControlMode.JOINTS)
    env_hw: gym.Env = FR3HW(env)
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 10
    gripper = rcsss.hw.FrankaHand(ip, gripper_cfg)
    env_hw = GripperWrapper(env_hw, gripper, binary=True)

    env_cg: gym.Env = CollisionGuard.env_from_xml_paths(
        env_hw,
        str(rcsss.scenes["fr3_empty_world"]),
        str(rcsss.scenes["lab"].parent / "fr3.urdf"),
        gripper=True,
        check_home_collision=False,
        camera=True,
        control_mode=control_mode,
        tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
    )
    env_rel = RelativeActionSpace(env_cg, max_mov=np.deg2rad(5))

    # if you have a realsense camera:
    # cameras = {
    #     "side": BaseCameraConfig(identifier="243522070385"),
    # }

    # cam_cfg = RealSenseSetConfig(
    #     cameras=cameras,
    #     resolution_width=1280,
    #     resolution_height=720,
    #     frame_rate=15,
    #     enable_imu=False,
    #     enable_ir=True,
    #     enable_ir_emitter=False,
    # )

    # camera_set = RealSenseCameraSet(cam_cfg)
    # camera_set.start()
    # env_rel = CameraSetWrapper(env_rel, camera_set)

    env_rel.reset()
    return env_rel


def sim_env_rel(control_mode: ControlMode) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    simulation = sim.Sim(rcsss.scenes["fr3_empty_world"])
    robot = rcsss.sim.FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    cfg.ik_duration_in_milliseconds = 300
    cfg.realtime = False
    robot.set_parameters(cfg)
    # env = FR3Env(robot, ControlMode.CARTESIAN_TQuart)
    env = FR3Env(robot, control_mode)
    env_sim = FR3Sim(env, simulation)
    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed), on_screen_render=False),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free), on_screen_render=True),
    }
    cam_cfg = SimCameraSetConfig(cameras=cameras, resolution_width=1280, resolution_height=720, frame_rate=10)
    camera_set = SimCameraSet(simulation, cam_cfg)
    env_cam: gym.Env = CameraSetWrapper(env_sim, camera_set)

    gripper_cfg = sim.FHConfig()
    gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
    env_cam = GripperWrapper(env_cam, gripper, binary=True)
    env_cam = RelativeActionSpace(env_cam, max_mov=np.deg2rad(5))
    env_cam.reset()
    return env_cam
