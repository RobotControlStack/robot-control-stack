import logging
from os import PathLike

import mujoco as mj
import numpy as np
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config, IKController
from rcsss._core.sim import CameraType, SimCameraSetConfig
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_sim_robot_cfg():
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    cfg.realtime = False
    return cfg


def default_fr3_hw_robot_cfg():
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.1
    robot_cfg.controller = IKController.robotics_library
    return robot_cfg


def default_fr3_hw_gripper_cfg():
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    return gripper_cfg


def default_realsense(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {name: BaseCameraConfig(identifier=id) for name, id in name2id.items()}
    cam_cfg = RealSenseSetConfig(
        cameras=cameras,
        resolution_width=1280,
        resolution_height=720,
        frame_rate=15,
        enable_imu=False,  # does not work with imu, why?
        enable_ir=True,
        enable_ir_emitter=False,
    )
    return RealSenseCameraSet(cam_cfg)


def default_fr3_sim_gripper_cfg():
    return sim.FHConfig()


def default_mujoco_cameraset_cfg():

    cameras = {
        "wrist": SimCameraConfig(identifier="eye-in-hand_0", type=int(CameraType.fixed)),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free)),
        # "bird_eye": SimCameraConfig(identifier="bird-eye-cam", type=int(CameraType.fixed)),
    }
    # 256x256 needed for VLAs
    return SimCameraSetConfig(
        cameras=cameras, resolution_width=256, resolution_height=256, frame_rate=10, physical_units=True
    )
        pose_offset = rcsss.common.Pose(translation=tcp_offset)
        cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()) * pose_offset
    return cfg


def get_urdf_path(urdf_path: str | PathLike | None, allow_none_if_not_found: bool = False) -> str | None:
    if urdf_path is None and "lab" in rcsss.scenes:
        urdf_path = rcsss.scenes["lab"].parent / "fr3.urdf"
        assert urdf_path.exists(), "Automatic deduced urdf path does not exist. Corrupted models directory."
        logger.info("Using automatic found urdf.")
    elif urdf_path is None and not allow_none_if_not_found:
        msg = "This pip package was not built with the UTN lab models, please pass the urdf and mjcf path to use simulation or collision guard."
        raise ValueError(msg)
    elif urdf_path is None:
        logger.warning("No urdf path was found. Proceeding, but set_cartesian methods will result in errors.")
    return str(urdf_path) if urdf_path is not None else None
