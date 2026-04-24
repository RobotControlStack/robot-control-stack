import logging

from digit_interface import Digit
from rcs._core.common import BaseCameraConfig
from rcs._core.sim import SimCameraConfig
from rcs.camera.digit_cam import DigitCam

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_sim_tilburg_hand_cfg() -> sim.SimTilburgHandConfig:
    return sim.SimTilburgHandConfig()


def default_digit(name2id: dict[str, str] | None, stream_name: str = "QVGA") -> DigitCam | None:
    if name2id is None:
        return None
    stream_dict = Digit.STREAMS[stream_name]
    cameras = {
        name: BaseCameraConfig(
            identifier=identifier,
            resolution_width=stream_dict["resolution"]["width"],
            resolution_height=stream_dict["resolution"]["height"],
            frame_rate=stream_dict["fps"]["30fps"],
        )
        for name, identifier in name2id.items()
    }
    return DigitCam(cameras=cameras)


def default_mujoco_cameraset_cfg() -> dict[str, SimCameraConfig]:
    # Kept for backwards compatibility in docs/comments while examples migrate.
    return {
        "wrist": SimCameraConfig(
            identifier="wrist_0",
            type=rcs._core.sim.CameraType.fixed,
            frame_rate=10,
            resolution_width=256,
            resolution_height=256,
        ),
        "default_free": SimCameraConfig(
            identifier="",
            type=rcs._core.sim.CameraType.default_free,
            frame_rate=10,
            resolution_width=256,
            resolution_height=256,
        ),
    }
