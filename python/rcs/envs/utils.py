import logging
from os import PathLike
from pathlib import Path

import mujoco as mj
import numpy as np
from digit_interface import Digit
from rcs._core.common import BaseCameraConfig
from rcs._core.sim import CameraType, SimCameraConfig
from rcs.camera.digit_cam import DigitCam

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_sim_robot_cfg(scene: str = "fr3_empty_world", idx: str = "0") -> sim.SimRobotConfig:
    robot_cfg = rcs.sim.SimRobotConfig()
    robot_cfg.tcp_offset = get_tcp_offset(rcs.scenes[scene].mjcf_scene)
    robot_cfg.realtime = False
    robot_cfg.robot_type = rcs.scenes[scene].robot_type
    robot_cfg.add_id(idx)
    robot_cfg.mjcf_scene_path = rcs.scenes[scene].mjb
    robot_cfg.kinematic_model_path = rcs.scenes[scene].mjcf_robot
    # robot_cfg.kinematic_model_path = rcs.scenes[scene].urdf
    return robot_cfg

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
            translation=tcp_offset_translation, rotation=tcp_offset_rotation_matrix.reshape((3, 3))  # type: ignore
        )
    except KeyError:
        msg = "No tcp offset found in the model. Using an identity transform as tcp offset."
        logging.warning(msg)
    return rcs.common.Pose()
