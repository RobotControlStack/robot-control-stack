import os
from pathlib import Path

from pydantic import BaseModel


class THConfig(BaseModel):
    """Config for the Tilburg hand"""

    binary_action: bool = True
    calibration_file_path: str = os.path.join(Path.home(), "tilburg_hand/calibration.json")
