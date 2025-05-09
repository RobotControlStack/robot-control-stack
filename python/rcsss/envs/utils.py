import logging
from os import PathLike
from pathlib import Path

import mujoco as mj
import numpy as np
import rcsss
from rcsss import sim
from rcsss._core.hw import FR3Config, IKSolver
from rcsss._core.sim import CameraType
from rcsss.camera.interface import BaseCameraConfig
from rcsss.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcsss.camera.sim import SimCameraConfig, SimCameraSetConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_sim_robot_cfg(mjcf: str | Path = "fr3_empty_world") -> sim.FR3Config:
    cfg = sim.FR3Config()
    cfg.tcp_offset = get_tcp_offset(mjcf)
    cfg.realtime = False
    return cfg


def default_fr3_hw_robot_cfg(async_control: bool = False):
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.1
    robot_cfg.ik_solver = IKSolver.rcs
    robot_cfg.async_control = async_control
    return robot_cfg


def default_fr3_hw_gripper_cfg(async_control: bool = False):
    gripper_cfg = rcsss.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    gripper_cfg.async_control = async_control
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
        "wrist": SimCameraConfig(identifier="wrist_0", type=int(CameraType.fixed)),
        "default_free": SimCameraConfig(identifier="", type=int(CameraType.default_free)),
        # "bird_eye": SimCameraConfig(identifier="bird_eye_cam", type=int(CameraType.fixed)),
    }
    # 256x256 needed for VLAs
    return SimCameraSetConfig(
        cameras=cameras, resolution_width=256, resolution_height=256, frame_rate=10, physical_units=True
    )


def get_tcp_offset(mjcf: str | Path):
    """Reads out tcp offset set in mjcf file.

    Convention: The tcp offset is stored in the model as a numeric attribute named "tcp_offset".

    Args:
        mjcf (str | PathLike): Path to the mjcf file.

    Returns:
        rcsss.common.Pose: The tcp offset.
    """
    if isinstance(mjcf, str):
        mjcf = Path(mjcf)
    mjmdl = rcsss.scenes.get(str(mjcf), mjcf)
    if mjmdl.suffix == ".xml":
        model = mj.MjModel.from_xml_path(str(mjmdl))
    elif mjmdl.suffix == ".mjb":
        model = mj.MjModel.from_binary_path(str(mjmdl))
    try:
        tcp_offset = np.array(model.numeric("tcp_offset").data)
        pose_offset = rcsss.common.Pose(translation=tcp_offset)
        return rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset()) * pose_offset
    except KeyError:
        msg = "No tcp offset found in the model. Using the default tcp offset."
        logging.warning(msg)
    return rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())


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
