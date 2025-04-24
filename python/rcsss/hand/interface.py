import os
from pathlib import Path

from pydantic import BaseModel


class THConfig(BaseModel):
    """Config for the Tilburg hand"""

    binary_action: bool = True
    calibration_file_path: str = os.path.join(Path.home(), "tilburg_hand/calibration.json")


class THMujocoConfig(BaseModel):
    """Config for the Mujoco Tilburg hand"""

    binary_action: bool = False
    mujoco_xml_path: str = os.path.join(Path.home(), "repos/tilburg-hand/src/tilburg_hand_urdf_mujoco/robot.xml")
