"""Factory for the `usb` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_usb_cam.camera import USBCameraConfig, USBCameraSet


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    if cfg.calibration != "dummy":
        msg = f"The usb backend supports only the 'dummy' calibration, got {cfg.calibration!r}"
        raise ValueError(msg)
    # The set reads USB specific fields (intrinsics, distortion) off every camera config.
    for name, camera_cfg in cfg.camera_cfgs.items():
        if not isinstance(camera_cfg, USBCameraConfig):
            msg = f"Expected USBCameraConfig for usb camera {name!r}, got {type(camera_cfg).__name__}"
            raise TypeError(msg)
    cameras = typing.cast(dict[str, USBCameraConfig], cfg.camera_cfgs)
    # calibration_strategy=None makes the set build dummy strategies itself.
    return typing.cast(HardwareCamera, USBCameraSet(cameras=cameras, calibration_strategy=None, **cfg.kwargs))
