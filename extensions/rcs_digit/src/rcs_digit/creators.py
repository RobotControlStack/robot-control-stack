"""Factory for the `digit` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_digit.camera import DigitCam


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    if cfg.calibration is not None:
        msg = "DIGIT sensors are not calibrated, `calibration` must be None"
        raise ValueError(msg)
    return typing.cast(HardwareCamera, DigitCam(cameras=cfg.camera_cfgs))
