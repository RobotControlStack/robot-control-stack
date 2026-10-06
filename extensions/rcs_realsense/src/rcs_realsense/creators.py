"""Factory for the `realsense` camera backend, registered as an `rcs.cameras` entry point."""

import typing
from collections.abc import Callable

from rcs.camera.hw import (
    CalibrationStrategy,
    DummyCalibrationStrategy,
    HardwareCamera,
    HardwareCameraCreatorConfig,
)
from rcs_realsense.calibration import FR3BaseArucoCalibration
from rcs_realsense.camera import RealSenseCameraSet

# Calibration ids a config may ask for, each a factory from camera name to strategy.
CALIBRATION_STRATEGIES: dict[str, Callable[[str], CalibrationStrategy]] = {
    "dummy": lambda _name: DummyCalibrationStrategy(),
    "fr3_base_aruco": FR3BaseArucoCalibration,
}


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    if cfg.calibration not in CALIBRATION_STRATEGIES:
        msg = f"Unknown realsense calibration {cfg.calibration!r}, available: {sorted(CALIBRATION_STRATEGIES)}"
        raise ValueError(msg)
    make_strategy = CALIBRATION_STRATEGIES[cfg.calibration]
    calibration_strategy = {name: make_strategy(name) for name in cfg.camera_cfgs}
    return typing.cast(
        HardwareCamera,
        RealSenseCameraSet(cameras=cfg.camera_cfgs, calibration_strategy=calibration_strategy, **cfg.kwargs),
    )
