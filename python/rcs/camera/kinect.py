import k4a
import numpy as np
from rcs._core.common import BaseCameraConfig
from rcs.camera.hw import HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame, IMUFrame


class KinectCamera(HardwareCamera):
    # ATTENTION: this code is untested
    def __init__(self, cameras: dict[str, BaseCameraConfig], include_imu: bool = False, timeout_ms: int = 2000) -> None:
        self.include_imu = include_imu
        self.timeout_ms = timeout_ms
        self.cameras = cameras
        assert len(self.cameras) == 1, "Kinect only supports one camera."
        self._camera_names = list(self.cameras.keys())

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    def open(self):
        self._device = k4a.Device.open()
        device_config = k4a.DEVICE_CONFIG_BGRA32_1080P_NFOV_2X2BINNED_FPS15
        if self._device is None:
            msg = "Failed to open device."
            raise OSError(msg)
        if self._device.start_cameras(device_config) != k4a.EStatus.SUCCEEDED:
            msg = "Failed to start cameras."
            raise OSError(msg)
        if self.include_imu and self._device.start_imu() != k4a.EWaitStatus.SUCCEEDED:
            msg = "Failed to start IMU."
            raise OSError(msg)

    def poll_frame(self, camera_name: str = "") -> Frame:
        assert camera_name == "kinect", "Kinect code only supports one camera."
        capture = self._device.get_capture(self.timeout_ms)
        if capture is None:
            msg = "Timeout error while waiting for camera sample."
            raise OSError(msg)
        assert capture.color is not None, "Color frame is None."
        camera_frame = CameraFrame(
            color=capture.color.data, ir=capture.ir.data, depth=capture.depth.data, temperature=capture.temperature
        )
        imu_frame = None
        if self.include_imu:
            imu_sample = self._device.get_imu_sample(self.timeout_ms)
            if imu_sample is None:
                msg = "Timeout error while waiting for IMU sample."
                raise OSError(msg)
            imu_sample_np = np.array((imu_sample.acc_sample.x, imu_sample.acc_sample.y, imu_sample.acc_sample.z))
            gyro_sample_np = np.array((imu_sample.gyro_sample.x, imu_sample.gyro_sample.y, imu_sample.gyro_sample.z))
            imu_frame = IMUFrame(
                accel=DataFrame(data=imu_sample_np, timestamp=float(imu_sample.acc_sample_usec)),
                gyro=DataFrame(data=gyro_sample_np, timestamp=float(imu_sample.gyro_sample_usec)),
                temperature=imu_sample.temperature,
            )
        return Frame(camera=camera_frame, imu=imu_frame, avg_timestamp=None)

    def config(self, camera_name) -> BaseCameraConfig:
        return self.cameras[camera_name]

    def close(self):
        if self._device is not None:
            self._device.stop_cameras()
            if self.include_imu:
                self._device.stop_imu()
            self._device = None
