"""Factory for the `zed` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import (
    CalibrationStrategy,
    DummyCalibrationStrategy,
    HardwareCamera,
    HardwareCameraCreatorConfig,
)
from rcs_zed.camera import ZEDCameraSet


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    if cfg.calibration != "dummy":
        msg = f"The zed backend supports only the 'dummy' calibration, got {cfg.calibration!r}"
        raise ValueError(msg)
    calibration_strategy = {
        name: typing.cast(CalibrationStrategy, DummyCalibrationStrategy()) for name in cfg.camera_cfgs
    }
    return typing.cast(
        HardwareCamera,
        ZEDCameraSet(cameras=cfg.camera_cfgs, calibration_strategy=calibration_strategy, **cfg.kwargs),
    )
