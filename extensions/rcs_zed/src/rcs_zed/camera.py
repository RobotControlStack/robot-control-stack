import copy
import logging
import threading
import typing
from dataclasses import dataclass
from time import time

import numpy as np
from rcs.camera.hw import CalibrationStrategy, DummyCalibrationStrategy, HardwareCamera
from rcs.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, IMUFrame

from rcs import common

try:
    from pyzed import sl
except ImportError:  # pragma: no cover - exercised via fake backend tests
    sl = None  # type: ignore[assignment]


@dataclass
class ZEDDeviceInfo:
    serial: str
    model: str
    has_depth: bool = True
    has_imu: bool = False


@dataclass
class ZEDFrameBundle:
    color: np.ndarray
    timestamp: float
    color_intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]]
    depth: np.ndarray | None = None
    accel: np.ndarray | None = None
    gyro: np.ndarray | None = None


def _intrinsics_matrix(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array(
        [
            [fx, 0, cx, 0],
            [0, fy, cy, 0],
            [0, 0, 1, 0],
        ],
        dtype=np.float64,
    )


class PyZEDCameraHandle:
    def __init__(self, camera: typing.Any, device_info: ZEDDeviceInfo, color_intrinsics: np.ndarray):
        self.camera = camera
        self.device_info = device_info
        self.color_intrinsics = color_intrinsics
        self.runtime_parameters = sl.RuntimeParameters()  # type: ignore[union-attr]
        self.image_mat = sl.Mat()  # type: ignore[union-attr]
        self.depth_mat = sl.Mat()  # type: ignore[union-attr]
        self.sensors_data = sl.SensorsData()  # type: ignore[union-attr]

    def _timestamp_seconds(self) -> float:
        try:
            timestamp = self.camera.get_timestamp(sl.TIME_REFERENCE.IMAGE)  # type: ignore[union-attr]
            if hasattr(timestamp, "get_nanoseconds"):
                return float(timestamp.get_nanoseconds()) * 1e-9
            if hasattr(timestamp, "get_microseconds"):
                return float(timestamp.get_microseconds()) * 1e-6
            if hasattr(timestamp, "get_milliseconds"):
                return float(timestamp.get_milliseconds()) * 1e-3
        except Exception:
            pass
        return time()

    def grab_frame(self) -> ZEDFrameBundle:
        err = self.camera.grab(self.runtime_parameters)
        if err != sl.ERROR_CODE.SUCCESS:  # type: ignore[union-attr]
            msg = f"Failed to grab ZED frame: {err}"
            raise RuntimeError(msg)

        self.camera.retrieve_image(self.image_mat, sl.VIEW.LEFT)  # type: ignore[union-attr]
        color_raw = np.array(self.image_mat.get_data(), copy=True)
        if color_raw.ndim != 3:
            msg = f"Unexpected ZED image shape {color_raw.shape}"
            raise RuntimeError(msg)
        color_rgb = color_raw[:, :, :3][:, :, ::-1] if color_raw.shape[2] == 4 else color_raw[:, :, ::-1]

        depth = None
        if self.device_info.has_depth:
            self.camera.retrieve_measure(self.depth_mat, sl.MEASURE.DEPTH)  # type: ignore[union-attr]
            depth_raw = np.array(self.depth_mat.get_data(), copy=True)
            if depth_raw.ndim > 2:
                depth_raw = depth_raw[:, :, 0]
            depth_m = np.nan_to_num(depth_raw, nan=0.0, posinf=0.0, neginf=0.0)
            depth_m = np.clip(depth_m, a_min=0.0, a_max=np.iinfo(np.uint16).max / BaseCameraSet.DEPTH_SCALE)
            depth = (depth_m * BaseCameraSet.DEPTH_SCALE).astype(np.uint16)

        accel = None
        gyro = None
        if self.device_info.has_imu:
            sensor_err = self.camera.get_sensors_data(self.sensors_data, sl.TIME_REFERENCE.IMAGE)  # type: ignore[union-attr]
            if sensor_err == sl.ERROR_CODE.SUCCESS:  # type: ignore[union-attr]
                imu_data = self.sensors_data.get_imu_data()
                if hasattr(imu_data, "get_linear_acceleration"):
                    accel = np.array(imu_data.get_linear_acceleration(), dtype=np.float64)
                if hasattr(imu_data, "get_angular_velocity"):
                    gyro = np.array(imu_data.get_angular_velocity(), dtype=np.float64)

        return ZEDFrameBundle(
            color=color_rgb,
            depth=depth,
            accel=accel,
            gyro=gyro,
            timestamp=self._timestamp_seconds(),
            color_intrinsics=self.color_intrinsics,
        )

    def close(self):
        self.camera.close()


class ZEDCameraSet(HardwareCamera):
    CALIBRATION_FRAME_SIZE = 30

    @staticmethod
    def _require_sdk():
        if sl is None:
            msg = (
                "The ZED SDK Python bindings are not available. Install the ZED SDK and ensure "
                "`import pyzed.sl as sl` works on this machine."
            )
            raise RuntimeError(msg)

    @staticmethod
    def _device_has_imu(device: typing.Any) -> bool:
        for attr in ("sensors_configuration", "sensors_conf"):
            sensors_conf = getattr(device, attr, None)
            if sensors_conf is None:
                continue
            camera_imu = getattr(sensors_conf, "camera_imu", None)
            if camera_imu is None:
                continue
            if hasattr(camera_imu, "available"):
                return bool(camera_imu.available)
            return True
        return False

    @staticmethod
    def _model_to_string(model: typing.Any) -> str:
        if hasattr(model, "name"):
            return str(model.name)
        return str(model)

    @staticmethod
    def _map_resolution(width: int, height: int):
        ZEDCameraSet._require_sdk()
        assert sl is not None
        mapping = {
            (2208, 1242): sl.RESOLUTION.HD2K,
            (1920, 1080): sl.RESOLUTION.HD1080,
            (1280, 720): sl.RESOLUTION.HD720,
            (672, 376): sl.RESOLUTION.VGA,
        }
        if (width, height) not in mapping:
            msg = f"Unsupported ZED resolution {width}x{height}. Use one of: {sorted(mapping)}"
            raise ValueError(msg)
        return mapping[(width, height)]

    @classmethod
    def enumerate_connected_devices(cls) -> dict[str, ZEDDeviceInfo]:
        cls._require_sdk()
        assert sl is not None
        devices: dict[str, ZEDDeviceInfo] = {}
        for device in sl.Camera.get_device_list():
            serial = str(device.serial_number)
            devices[serial] = ZEDDeviceInfo(
                serial=serial,
                model=cls._model_to_string(device.camera_model),
                has_depth=True,
                has_imu=cls._device_has_imu(device),
            )
        return devices

    @classmethod
    def open_camera(
        cls,
        config: common.BaseCameraConfig,
        *,
        enable_depth: bool,
        enable_imu: bool,
    ) -> PyZEDCameraHandle:
        cls._require_sdk()
        assert sl is not None

        init = sl.InitParameters()
        init.camera_resolution = cls._map_resolution(config.resolution_width, config.resolution_height)
        init.camera_fps = config.frame_rate
        init.coordinate_units = sl.UNIT.METER
        init.depth_mode = sl.DEPTH_MODE.NONE if not enable_depth else sl.DEPTH_MODE.QUALITY
        init.sdk_verbose = False
        init.set_from_serial_number(int(config.identifier))

        camera = sl.Camera()
        err = camera.open(init)
        if err != sl.ERROR_CODE.SUCCESS:
            msg = f"Could not open ZED camera {config.identifier}: {err}"
            raise RuntimeError(msg)

        information = camera.get_camera_information()
        calibration = information.camera_configuration.calibration_parameters
        left_cam = calibration.left_cam
        info = ZEDDeviceInfo(
            serial=str(config.identifier),
            model=cls._model_to_string(information.camera_model),
            has_depth=enable_depth,
            has_imu=enable_imu and cls._device_has_imu(information),
        )
        intrinsics = _intrinsics_matrix(left_cam.fx, left_cam.fy, left_cam.cx, left_cam.cy)
        return PyZEDCameraHandle(camera=camera, device_info=info, color_intrinsics=intrinsics)

    def __init__(
        self,
        cameras: dict[str, common.BaseCameraConfig],
        calibration_strategy: dict[str, CalibrationStrategy] | None = None,
        enable_depth: bool = True,
        enable_imu: bool = True,
    ) -> None:
        self.cameras = cameras
        if calibration_strategy is None:
            calibration_strategy = {camera_name: DummyCalibrationStrategy() for camera_name in cameras}
        self.calibration_strategy = calibration_strategy
        self.enable_depth = enable_depth
        self.enable_imu = enable_imu
        self._logger = logging.getLogger(__name__)
        self._camera_names = list(self.cameras.keys())
        self._available_devices: dict[str, ZEDDeviceInfo] = {}
        self._enabled_devices: dict[str, PyZEDCameraHandle] = {}
        self._frame_buffer_lock: dict[str, threading.Lock] = {}
        self._frame_buffer: dict[str, list[Frame]] = {}

        assert (
            len({camera.resolution_width for camera in self.cameras.values()}) == 1
            and len({camera.resolution_height for camera in self.cameras.values()}) == 1
            and len({camera.frame_rate for camera in self.cameras.values()}) == 1
        ), "All cameras must have the same resolution and frame rate."

    @property
    def camera_names(self) -> list[str]:
        return self._camera_names

    def config(self, camera_name) -> common.BaseCameraConfig:
        return self.cameras[camera_name]

    def update_available_devices(self):
        self._available_devices = self.enumerate_connected_devices()

    def open(self):
        self.update_available_devices()
        self._enabled_devices = {}
        self._frame_buffer = {}
        self._frame_buffer_lock = {}

        for camera_name, camera_cfg in self.cameras.items():
            if camera_cfg.identifier not in self._available_devices:
                msg = f"ZED device {camera_name} with serial {camera_cfg.identifier} not found."
                raise RuntimeError(msg)

            device_info = self._available_devices[camera_cfg.identifier]
            opened = self.open_camera(
                camera_cfg,
                enable_depth=self.enable_depth and device_info.has_depth,
                enable_imu=self.enable_imu and device_info.has_imu,
            )
            self._enabled_devices[camera_name] = opened
            self._frame_buffer[camera_name] = []
            self._frame_buffer_lock[camera_name] = threading.Lock()

    def poll_frame(self, camera_name: str) -> Frame:
        assert camera_name in self.camera_names, f"Camera {camera_name} not found in the enabled devices"
        device = self._enabled_devices[camera_name]
        bundle = device.grab_frame()
        extrinsics = self.calibration_strategy[camera_name].get_extrinsics()

        color = DataFrame(
            data=bundle.color,
            timestamp=bundle.timestamp,
            intrinsics=bundle.color_intrinsics,
            extrinsics=extrinsics,
        )
        depth = None
        if bundle.depth is not None:
            depth = DataFrame(
                data=bundle.depth,
                timestamp=bundle.timestamp,
                intrinsics=bundle.color_intrinsics,
                extrinsics=extrinsics,
            )

        accel = DataFrame(data=bundle.accel, timestamp=bundle.timestamp) if bundle.accel is not None else None
        gyro = DataFrame(data=bundle.gyro, timestamp=bundle.timestamp) if bundle.gyro is not None else None

        frame = Frame(
            camera=CameraFrame(color=color, depth=depth),
            imu=IMUFrame(accel=accel, gyro=gyro) if (accel is not None or gyro is not None) else None,
            avg_timestamp=bundle.timestamp,
        )

        with self._frame_buffer_lock[camera_name]:
            if len(self._frame_buffer[camera_name]) >= self.CALIBRATION_FRAME_SIZE:
                self._frame_buffer[camera_name].pop(0)
            self._frame_buffer[camera_name].append(copy.deepcopy(frame))
        return frame

    def close(self):
        for device in self._enabled_devices.values():
            device.close()
        self._enabled_devices = {}

    def calibrate(self) -> bool:
        for camera_name in self.cameras:
            device = self._enabled_devices.get(camera_name)
            if device is None:
                msg = f"Camera {camera_name} must be opened before calibration."
                raise RuntimeError(msg)
            if not self.calibration_strategy[camera_name].calibrate(
                intrinsics=device.color_intrinsics,
                samples=self._frame_buffer[camera_name],
                lock=self._frame_buffer_lock[camera_name],
            ):
                self._logger.warning("Calibration of camera %s failed.", camera_name)
                return False
        self._logger.info("Calibration successful.")
        return True
