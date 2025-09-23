import logging
from os import PathLike

from digit_interface import Digit
from rcs._core.common import BaseCameraConfig
from rcs._core.sim import CameraType, SimCameraConfig
from rcs.camera.digit_cam import DigitCam
from rcs.hand.tilburg_hand import THConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_sim_robot_cfg(scene: str = "fr3_empty_world", idx: str = "0") -> sim.SimRobotConfig:
    robot_cfg = rcs.sim.SimRobotConfig()
    robot_cfg.realtime = False
    robot_cfg.robot_type = rcs.scenes[scene].robot_type
    robot_cfg.add_id(idx)
    robot_cfg.mjcf_scene_path = rcs.scenes[scene].mjb
    robot_cfg.kinematic_model_path = rcs.scenes[scene].mjcf_robot
    # robot_cfg.kinematic_model_path = rcs.scenes[scene].urdf
    return robot_cfg


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
