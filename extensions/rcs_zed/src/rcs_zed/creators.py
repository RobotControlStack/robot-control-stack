"""Factory for the `zed` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_zed.camera import ZEDCameraSet


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    # calibration=None leaves the set to build identity strategies for every camera.
    return typing.cast(
        HardwareCamera,
        ZEDCameraSet(cameras=cfg.camera_cfgs, calibration_strategy=cfg.calibration, **cfg.kwargs),
    )
