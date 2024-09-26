import logging
import sys

import gymnasium as gym
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config
from rcsss._core.sim import CameraType
from rcsss.camera.hw import BaseHardwareCameraSet
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig, SimCameraSet, SimCameraSetConfig
from rcsss.envs.base import (
    CameraSetWrapper,
    ControlMode,
    FR3Env,
    GripperWrapper,
    LimitedJointsRelDictType,
    ObsArmsGrCam,
    RelativeActionSpace,
)
from rcsss.envs.hw import FR3HW
from rcsss.envs.sim import CollisionGuard, FR3Sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_hw_robot_cfg():
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.2
    return robot_cfg


def default_fr3_hw_gripper_cfg():
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    return gripper_cfg


def default_realsense(name2id: dict[str, str]):
    cameras = {name: BaseCameraConfig(identifier=id) for name, id in name2id.items()}
    cam_cfg = RealSenseSetConfig(
        cameras=cameras,
        resolution_width=1280,
        resolution_height=720,
        frame_rate=15,
        enable_imu=False,
        enable_ir=True,
        enable_ir_emitter=False,
    )
    return RealSenseCameraSet(cam_cfg)


def fr3_hw_env(
    ip: str,
    control_mode: ControlMode,
    robot_cfg: rcsss.hw.FR3Config,
    collision_guard: bool = True,
    gripper_cfg: rcsss.hw.FHConfig | None = None,
    camera_set: BaseHardwareCameraSet | None = None,
    max_relative_movement: float | None = None,
) -> gym.Env:
    if "lab" not in rcsss.scenes:
        # TODO: mujoco xml and urdf as arguments
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    robot = rcsss.hw.FR3(ip, str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    robot.set_parameters(robot_cfg)

    env: gym.Env = FR3Env(robot, ControlMode.JOINTS if collision_guard else control_mode)

    env = FR3HW(env)
    if gripper_cfg is not None:
        gripper = rcsss.hw.FrankaHand(ip, gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if camera_set is not None:
        camera_set.start()
        env = CameraSetWrapper(env, camera_set)

    if collision_guard:
        env = CollisionGuard.env_from_xml_paths(
            env,
            str(rcsss.scenes["fr3_empty_world"]),
            str(rcsss.scenes["lab"].parent / "fr3.urdf"),
            gripper=True,
            check_home_collision=False,
            camera=True,
            control_mode=control_mode,
            tcp_offset=rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()),
        )
    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement)

    return env


def default_fr3_sim_robot_cfg():
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    cfg.ik_duration_in_milliseconds = 300
    cfg.realtime = False
    return cfg


def default_fr3_sim_gripper_cfg():
    return sim.FHConfig()


def default_mujoco_cameraset_cfg():

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed)),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free)),
    }
    return SimCameraSetConfig(cameras=cameras, resolution_width=1280, resolution_height=720, frame_rate=10)


def fr3_sim_env(
    control_mode: ControlMode,
    robot_cfg: rcsss.sim.FR3Config,
    gripper_cfg: rcsss.sim.FHConfig | None = None,
    camera_set_cfg: SimCameraSetConfig | None = None,
    max_relative_movement: float | None = None,
) -> gym.Env[ObsArmsGrCam, LimitedJointsRelDictType]:
    if "lab" not in rcsss.scenes:
        logger.error("This pip package was not built with the UTN lab models, aborting.")
        sys.exit()
    simulation = sim.Sim(rcsss.scenes["fr3_empty_world"])
    robot = rcsss.sim.FR3(simulation, "0", str(rcsss.scenes["lab"].parent / "fr3.urdf"))
    robot.set_parameters(robot_cfg)
    env: gym.Env = FR3Env(robot, control_mode)
    env = FR3Sim(env, simulation)

    if camera_set_cfg is not None:
        camera_set = SimCameraSet(simulation, camera_set_cfg)
        env = CameraSetWrapper(env, camera_set)

    if gripper_cfg is not None:
        gripper = sim.FrankaHand(simulation, "0", gripper_cfg)
        env = GripperWrapper(env, gripper, binary=True)

    if max_relative_movement is not None:
        env = RelativeActionSpace(env, max_mov=max_relative_movement)

    return env
