import logging

from rcs._core.sim import SimCameraConfig

import rcs
from rcs import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_sim_tilburg_hand_cfg() -> sim.SimTilburgHandConfig:
    return sim.SimTilburgHandConfig()


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
