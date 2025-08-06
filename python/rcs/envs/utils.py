import logging
from os import PathLike
from pathlib import Path

import mujoco as mj
import numpy as np
from digit_interface import Digit
from rcs._core.common import BaseCameraConfig
from rcs._core.sim import CameraType, SimCameraConfig
from rcs.camera.digit_cam import DigitCam
from rcs.hand.tilburg_hand import THConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_sim_robot_cfg(mjcf: str | Path = "fr3_empty_world", idx: str = "0") -> sim.SimRobotConfig:
    cfg = sim.SimRobotConfig()
    cfg.tcp_offset = get_tcp_offset(mjcf)
    cfg.realtime = False
    cfg.robot_type = rcs.common.RobotType.FR3
    cfg.add_id(idx)
    return cfg


def default_tilburg_hw_hand_cfg(file: str | PathLike | None = None) -> THConfig:
    hand_cfg = THConfig()
    hand_cfg.grasp_percentage = 1.0
    hand_cfg.calibration_file = str(file) if isinstance(file, PathLike) else file
    return hand_cfg


def default_sim_gripper_cfg(idx: str = "0") -> sim.SimGripperConfig:
    cfg = sim.SimGripperConfig()
    cfg.add_id(idx)
    return cfg


def default_sim_tilburg_hand_cfg() -> sim.SimTilburgHandConfig:
    return sim.SimTilburgHandConfig()


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
            msg = f"Expected .mjb, .mjcf or.xml, got {mjcf.suffix} and {mjcf}"
            raise AssertionError(msg)
    try:
        tcp_offset_translation = np.array(model.numeric("tcp_offset_translation").data)
        tcp_offset_rotation_matrix = np.array(model.numeric("tcp_offset_rotation_matrix").data)
        return rcs.common.Pose(
            translation=tcp_offset_translation, rotation=tcp_offset_rotation_matrix.reshape((3, 3))
        )  #  type: ignore
    except KeyError:
        msg = "No tcp offset found in the model. Using the default tcp offset."
        logging.info(msg)
    return rcs.common.Pose()
