"""Factory for the `digit` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_digit.camera import DigitCam


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    if cfg.calibration != "dummy":
        msg = f"DIGIT sensors are not calibrated, got calibration {cfg.calibration!r}"
        raise ValueError(msg)
    return typing.cast(HardwareCamera, DigitCam(cameras=cfg.camera_cfgs))
