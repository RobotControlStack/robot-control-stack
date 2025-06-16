import logging
from os import PathLike
from pathlib import Path

import mujoco as mj
import numpy as np
import rcs
from digit_interface import Digit
from rcs import sim
from rcs._core.common import BaseCameraConfig
from rcs._core.hw import FR3Config, IKSolver
from rcs._core.sim import CameraType, SimCameraConfig
from rcs.camera.digit_cam import DigitCam
from rcs.camera.realsense import RealSenseCameraSet
from rcs.hand.tilburg_hand import THConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_sim_robot_cfg(mjcf: str | Path = "fr3_empty_world", idx: str = "0") -> sim.SimRobotConfig:
    cfg = sim.SimRobotConfig()
    cfg.tcp_offset = get_tcp_offset(mjcf)
    cfg.realtime = False
    cfg.robot_type = rcs.common.RobotType.FR3
    # cfg.add_id(idx)
    return cfg


def default_fr3_hw_robot_cfg(async_control: bool = False) -> FR3Config:
    robot_cfg = FR3Config()
    robot_cfg.tcp_offset = rcs.common.Pose(rcs.common.FrankaHandTCPOffset())
    robot_cfg.speed_factor = 0.1
    robot_cfg.ik_solver = IKSolver.rcs_ik
    robot_cfg.async_control = async_control
    return robot_cfg


def default_fr3_hw_gripper_cfg(async_control: bool = False) -> rcs.hw.FHConfig:
    gripper_cfg = rcs.hw.FHConfig()
    gripper_cfg.epsilon_inner = gripper_cfg.epsilon_outer = 0.1
    gripper_cfg.speed = 0.1
    gripper_cfg.force = 30
    gripper_cfg.async_control = async_control
    return gripper_cfg


def default_tilburg_hw_hand_cfg(file: str | PathLike | None = None) -> THConfig:
    hand_cfg = THConfig()
    hand_cfg.grasp_percentage = 1.0
    hand_cfg.calibration_file = str(file) if isinstance(file, PathLike) else file
    return hand_cfg


def default_realsense(name2id: dict[str, str] | None) -> RealSenseCameraSet | None:
    if name2id is None:
        return None
    cameras = {
        name: BaseCameraConfig(identifier=id, resolution_width=1280, resolution_height=720, frame_rate=30)
        for name, id in name2id.items()
    }
    return RealSenseCameraSet(cameras=cameras)


def default_fr3_sim_gripper_cfg(idx: str = "0") -> sim.SimGripperConfig:
    cfg = sim.SimGripperConfig()
    # cfg.add_id(idx)
    return cfg


def default_digit(name2id: dict[str, str] | None, stream_name: str = "QVGA") -> DigitCam | None:
    if name2id is None:
        return None
    stream_dict = Digit.STREAMS[stream_name]
    cameras = {
        name: BaseCameraConfig(
            identifier=id,
            resolution_width=stream_dict["resolution"]["width"],
            resolution_height=stream_dict["resolution"]["height"],
            frame_rate=stream_dict["fps"]["30fps"],
        )
        for name, id in name2id.items()
    }
    return DigitCam(cameras=cameras)


def default_mujoco_cameraset_cfg() -> dict[str, SimCameraConfig]:
    # 256x256 needed for VLAs
    return {
        "wrist": SimCameraConfig(
            identifier="wrist_0", type=CameraType.fixed, frame_rate=10, resolution_width=256, resolution_height=256
        ),
        "default_free": SimCameraConfig(
            identifier="", type=CameraType.default_free, frame_rate=10, resolution_width=256, resolution_height=256
        ),
        # "bird_eye": SimCameraConfig(identifier="bird_eye_cam", type=int(CameraType.fixed), frame_rate=10, resolution_width=256, resolution_height=256),
    }


def get_tcp_offset(mjcf: str | Path) -> rcs.common.Pose:
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
