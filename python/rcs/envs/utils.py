import logging
from pathlib import Path

import mujoco as mj
import numpy as np
import rcs
from rcs import sim
from rcs._core.hw import FR3Config, IKSolver
from rcs._core.sim import CameraType
from rcs.camera.interface import BaseCameraConfig
from rcs.camera.realsense import RealSenseCameraSet, RealSenseSetConfig
from rcs.camera.sim import SimCameraConfig, SimCameraSetConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_sim_robot_cfg(mjcf: str | Path = "fr3_empty_world") -> sim.SimRobotConfig:
    cfg = sim.SimRobotConfig()
    cfg.tcp_offset = get_tcp_offset(mjcf)
    cfg.realtime = False
    return cfg


def default_fr3_hw_robot_cfg(async_control: bool = False):
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.1
    robot_cfg.ik_solver = IKSolver.rcs_ik
    robot_cfg.async_control = async_control
    return robot_cfg


def default_fr3_hw_gripper_cfg(async_control: bool = False):
    gripper_cfg = rcs.hw.FHConfig()
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
    return sim.SimGripperConfig()


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
        rcs.common.Pose: The tcp offset.
    """
    if mjcf in rcs.scenes:
        model = mj.MjModel.from_binary_path(str(rcs.scenes[str(mjcf)]["mjb"]))
    else:
        mjcf = Path(mjcf)
        if mjcf.suffix in (".xml", ".mjcf"):
            model = mj.MjModel.from_xml_path(str(mjcf))
        elif mjcf.suffix == ".mjb":
            model = mj.MjModel.from_binary_path(str(mjcf))
        else:
            msg = f"Expected .mjb, .mjcf or.xml, got {mjcf.suffix}"
            raise AssertionError(msg)
    try:
        tcp_offset = np.array(model.numeric("tcp_offset").data)
        pose_offset = rcs.common.Pose(translation=tcp_offset)
        return rcs.common.Pose(rcs.common.FrankaHandTCPOffset()) * pose_offset
    except KeyError:
        msg = "No tcp offset found in the model. Using the default tcp offset."
        logging.info(msg)
    return rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
