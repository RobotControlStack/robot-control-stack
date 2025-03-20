import logging
from os import PathLike

import numpy as np
import rcsss
from rcsss import sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def default_fr3_sim_robot_cfg():
    cfg = sim.FR3Config()
    cfg.tcp_offset = rcsss.common.Pose(rcsss.common.FrankaHandTCPOffset())
    cfg.realtime = False
    return cfg


def set_tcp_offset(cfg: sim.FR3Config, sim: sim.Sim):
    tcp_offset = np.array([0, 0, 0])
    try:
        tcp_offset = np.array(sim.model.numeric("tcp_offset").data)
    except KeyError:
        print("Using the default tcp offset")
    if tcp_offset is not None:
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
