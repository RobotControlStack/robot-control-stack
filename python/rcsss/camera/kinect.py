import typing

import k4a
import numpy as np
from rcsss.camera.interface import (
    Camera,
    CameraFrame,
    Frame,
    GenericCameraConfig,
    IMUFrame,
)


class KinectConfig(GenericCameraConfig):
    device_config: k4a.DeviceConfiguration = k4a.DEVICE_CONFIG_BGRA32_2160P_WFOV_2X2BINNED_FPS15
    include_imu: bool = False
    timeout_ms: int = 2000


class KinectCamera(Camera):
    def __init__(self, cfg: KinectConfig) -> None:
        self._cfg = cfg
        self._device = k4a.Device.open()
        if self._device is None:
            msg = "Failed to open device."
            raise OSError(msg)
        if self._device.start_cameras(self._cfg.device_config) != k4a.EStatus.SUCCEEDED:
            msg = "Failed to start cameras."
            raise OSError(msg)
        if self._cfg.include_imu and self._device.start_imu() != k4a.EWaitStatus.SUCCEEDED:
            msg = "Failed to start IMU."
            raise OSError(msg)

    @property
    def config(self) -> GenericCameraConfig:
        return self._cfg

    @config.setter
    def config(self, cfg: GenericCameraConfig) -> None:
        self._cfg = typing.cast(KinectConfig, cfg)

    def get_current_frame(self) -> Frame:
        capture = self._device.get_capture(self._cfg.timeout_ms)
        if capture is None:
            msg = "Timeout error while waiting for camera sample."
            raise OSError(msg)
        camera_frame = CameraFrame(
            color=capture.color.data, ir=capture.ir.data, depth=capture.depth.data, temperature=capture.temperature
        )
        imu_frame = None
        if self._cfg.include_imu:
            imu_sample = self._device.get_imu_sample(self._cfg.timeout_ms)
            if imu_sample is None:
                msg = "Timeout error while waiting for IMU sample."
                raise OSError(msg)
            imu_sample_np = np.array((imu_sample.acc_sample.x, imu_sample.acc_sample.y, imu_sample.acc_sample.z))
            gyro_sample_np = np.array((imu_sample.gyro_sample.x, imu_sample.gyro_sample.y, imu_sample.gyro_sample.z))
            imu_frame = IMUFrame(
                acc_sample=imu_sample_np,
                acc_sample_usec=imu_sample.acc_sample_usec,
                gyro_sample=gyro_sample_np,
                gyro_sample_usec=imu_sample.gyro_sample_usec,
                temperature=imu_sample.temperature,
            )
        return Frame(camera=camera_frame, imu=imu_frame)
