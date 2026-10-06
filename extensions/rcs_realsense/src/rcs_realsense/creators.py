"""Factory for the `realsense` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_realsense.camera import RealSenseCameraSet


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    # calibration=None leaves the set to build identity strategies for every camera.
    return typing.cast(
        HardwareCamera,
        RealSenseCameraSet(cameras=cfg.camera_cfgs, calibration_strategy=cfg.calibration, **cfg.kwargs),
    )
