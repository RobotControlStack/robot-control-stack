import logging
from dataclasses import dataclass
from pathlib import Path
from os import PathLike
from time import sleep

from rcs_tilburg_hand.tilburg_hand import TilburgHand, THConfig


def default_tilburg_hw_hand_cfg(file: str | PathLike | None = None) -> THConfig:
    hand_cfg = THConfig()
    hand_cfg.grasp_percentage = 1.0
    hand_cfg.calibration_file = str(file) if isinstance(file, PathLike) else file
    return hand_cfg

if __name__ == "__main__":
    # Example usage of the Tilburg Hand configuration
    config = default_tilburg_hw_hand_cfg(file=Path("~/tilburg_hand/calibration.json"))
    hand = TilburgHand(cfg=config, verbose=True)
    logger = logging.getLogger(__name__)
    # Log the configuration
    logger.info(f"Tilburg Hand Configuration: {hand.config}")
    hand.grasp()
    # Perform operations with the hand
    # For example, you can call methods like hand.grasp() or hand.reset()
    sleep(5)  # Simulate some operation time
    # Disconnect when done
    hand._motors.disconnect()
    logger.info("Disconnected from the motors' board.")