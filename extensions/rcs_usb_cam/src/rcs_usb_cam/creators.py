"""Factory for the `usb` camera backend, registered as an `rcs.cameras` entry point."""

import typing

from rcs.camera.hw import HardwareCamera, HardwareCameraCreatorConfig
from rcs_usb_cam.camera import USBCameraConfig, USBCameraSet


def create_camera_set(cfg: HardwareCameraCreatorConfig) -> HardwareCamera:
    # The set reads USB specific fields (intrinsics, distortion) off every camera config.
    for name, camera_cfg in cfg.camera_cfgs.items():
        if not isinstance(camera_cfg, USBCameraConfig):
            msg = f"Expected USBCameraConfig for usb camera {name!r}, got {type(camera_cfg).__name__}"
            raise TypeError(msg)
    cameras = typing.cast(dict[str, USBCameraConfig], cfg.camera_cfgs)
    # calibration=None leaves the set to build identity strategies for every camera.
    return typing.cast(
        HardwareCamera, USBCameraSet(cameras=cameras, calibration_strategy=cfg.calibration, **cfg.kwargs)
    )
