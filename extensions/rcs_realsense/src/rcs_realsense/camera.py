import copy
import logging
import threading
import typing
from dataclasses import dataclass
from time import sleep

import numpy as np
import pyrealsense2 as rs
from rcs.camera.hw import CalibrationStrategy, DummyCalibrationStrategy, HardwareCamera
from rcs.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, IMUFrame

from rcs import common


@dataclass
class RealSenseDeviceInfo:
    product_line: str
    serial: str


@dataclass
class RealSenseDevicePipeline:
    pipeline: rs.pipeline
    pipeline_profile: rs.pipeline_profile
    camera: RealSenseDeviceInfo
    depth_scale: float | None = None
    color_intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]] | None = None
    depth_intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]] | None = None
    depth_to_color: common.Pose | None = None


class RealSenseCameraSet(HardwareCamera):
    TIMESTAMP_FACTOR = 1e-3
    CALIBRATION_FRAME_SIZE = 30

    def __init__(
        self,
        cameras: dict[str, common.BaseCameraConfig],
        calibration_strategy: dict[str, CalibrationStrategy] | None = None,
        enable_ir_emitter: bool = False,
        enable_ir: bool = False,
        laser_power: int = 330,
        enable_imu: bool = False,
    ) -> None:
        self.enable_ir_emitter = enable_ir_emitter
        self.enable_ir = enable_ir
        self.laser_power = laser_power
        self.enable_imu = enable_imu
        self.cameras = cameras
        if calibration_strategy is None:
            calibration_strategy = {camera_name: DummyCalibrationStrategy() for camera_name in cameras}
        self.calibration_strategy = calibration_strategy
        self._logger = logging.getLogger(__name__)
        assert (
            len({camera.resolution_width for camera in self.cameras.values()}) == 1
            and len({camera.resolution_height for camera in self.cameras.values()}) == 1
            and len({camera.frame_rate for camera in self.cameras.values()}) == 1
        ), "All cameras must have the same resolution and frame rate."
        sample_camera_config = next(iter(self.cameras.values()))
        self.resolution_width = sample_camera_config.resolution_width
        self.resolution_height = sample_camera_config.resolution_height
        self.frame_rate = sample_camera_config.frame_rate
        self._frame_buffer_lock: dict[str, threading.Lock] = {}
        self._frame_buffer: dict[str, list] = {}

        self.D400_config = rs.config()
        self.D400_config.enable_stream(
            rs.stream.depth,
            self.resolution_width,
            self.resolution_height,
            rs.format.z16,
            self.frame_rate,
        )
        self.D400_config.enable_stream(
            rs.stream.color,
            self.resolution_width,
            self.resolution_height,
            rs.format.bgr8,
            self.frame_rate,
        )
        if self.enable_ir:
            self.D400_config.enable_stream(
                rs.stream.infrared,
                1,
                self.resolution_width,
                self.resolution_height,
                rs.format.y8,
                self.frame_rate,
            )
        if self.enable_imu:
            # TODO(juelg): does not work work at the moment: "Couldnt resolve requests"
            # https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/
            # Accelerometer available FPS: {63, 250}Hz
            self.D400_config.enable_stream(
                rs.stream.accel,
                rs.format.motion_xyz32f,
                250,
            )
            # Gyroscope available FPS: {200,400}Hz
            self.D400_config.enable_stream(
                rs.stream.gyro,
                rs.format.motion_xyz32f,
                200,
            )
        self._available_devices: dict[str, RealSenseDeviceInfo] = {}
        self._enabled_devices: dict[str, RealSenseDevicePipeline] = {}  # serial numbers of te enabled devices
        self._camera_names = list(self.cameras.keys())

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    def config(self, camera_name) -> common.BaseCameraConfig:
        return self.cameras[camera_name]

    def open(self):
        self._available_devices = {}
        self.update_available_devices()
        self._enabled_devices = {}  # serial numbers of te enabled devices
        self.enable_devices({key: value.identifier for key, value in self.cameras.items()}, self.enable_ir_emitter)

    def update_available_devices(self):
        self._available_devices = self.enumerate_connected_devices(rs.context())

    def enable_devices(self, devices_to_enable: dict[str, str], enable_ir_emitter: bool = False):
        """
        Enable the Intel RealSense Devices which are connected to the PC

        Parameters:
        -----------
        devices_to_enable : dict
                            Dictionary with readable name and serial number of the devices to be enabled
        enable_ir_emitter : bool
                            Enable/Disable the IR-Emitter of the device

        """
        for device_name, device_serial in devices_to_enable.items():
            assert (
                device_serial in self._available_devices
            ), f"Device {device_name} not found. Check if it is connected."
            self.enable_device(device_name, self._available_devices[device_serial], enable_ir_emitter)

    def enable_device(self, camera_name: str, device_info: RealSenseDeviceInfo, enable_ir_emitter: bool = False):
        """
        Enable an Intel RealSense Device

        Parameters:
        -----------
        device_info     : Tuple of strings (serial_number, product_line)
                            Serial number and product line of the realsense device
        enable_ir_emitter : bool
                            Enable/Disable the IR-Emitter of the device

        """
        pipeline = rs.pipeline()

        if device_info.product_line == "D400":
            # Enable D400 device
            self.D400_config.enable_device(device_info.serial)
            pipeline_profile = pipeline.start(self.D400_config)
        else:
            msg = "unknown product line {device_info.product_line}"
            raise RuntimeError(msg)

        # Set the acquisition parameters
        sensor = pipeline_profile.get_device().first_depth_sensor()
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
            sensor.set_option(rs.option.laser_power, self.laser_power)

        depth_vp = pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile()
        color_vp = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()

        rs_color_intrinsics = color_vp.get_intrinsics()
        color_intrinsics = np.array(
            [
                [rs_color_intrinsics.fx, 0, (rs_color_intrinsics.width - 1) / 2, 0],
                [0, rs_color_intrinsics.fy, (rs_color_intrinsics.height - 1) / 2, 0],
                [0, 0, 1, 0],
            ]
        )
        rs_depth_intrinsics = depth_vp.get_intrinsics()
        depth_intrinsics = np.array(
            [
                [rs_depth_intrinsics.fx, 0, (rs_depth_intrinsics.width - 1) / 2, 0],
                [0, rs_depth_intrinsics.fy, (rs_depth_intrinsics.height - 1) / 2, 0],
                [0, 0, 1, 0],
            ]
        )

        depth_to_color = depth_vp.get_extrinsics_to(color_vp)

        self._enabled_devices[camera_name] = RealSenseDevicePipeline(
            pipeline,
            pipeline_profile,
            device_info,
            depth_scale=sensor.get_depth_scale(),
            color_intrinsics=color_intrinsics,  # type: ignore
            depth_intrinsics=depth_intrinsics,  # type: ignore
            depth_to_color=common.Pose(
                translation=depth_to_color.translation, rotation=np.array(depth_to_color.rotation).reshape(3, 3)  # type: ignore
            ),
        )

        self._frame_buffer[camera_name] = []
        self._frame_buffer_lock[camera_name] = threading.Lock()
        self._logger.debug("Enabled device %s (%s)", device_info.serial, device_info.product_line)

    @staticmethod
    def enumerate_connected_devices(context: rs.context) -> dict[str, RealSenseDeviceInfo]:
        """
        Enumerate the connected Intel RealSense devices

        Parameters:
        -----------
        context 	   : rs.context()
                        The context created for using the realsense library

        Return:
        -----------
        connect_device : array
                        Array of (serial, product-line) tuples of devices which are connected to the PC

        """
        connect_device: dict[str, RealSenseDeviceInfo] = {}

        d: rs.device
        for d in context.devices:
            if d.get_info(rs.camera_info.name).lower() != "platform camera":
                serial = d.get_info(rs.camera_info.serial_number)
                product_line = d.get_info(rs.camera_info.product_line)
                device_info = RealSenseDeviceInfo(serial=serial, product_line=product_line)
                connect_device[serial] = device_info
        return connect_device

    def poll_frame(self, camera_name: str) -> Frame:
        assert camera_name in self.camera_names, f"Camera {camera_name} not found in the enabled devices"
        device = self._enabled_devices[camera_name]

        streams = device.pipeline_profile.get_streams()
        frameset = device.pipeline.wait_for_frames()

        color: DataFrame | None = None
        ir: DataFrame | None = None
        depth: DataFrame | None = None
        accel: DataFrame | None = None
        gyro: DataFrame | None = None

        def to_numpy(frame: rs.frame) -> np.ndarray:
            return np.asanyarray(frame.get_data()).copy()

        def to_ts(frame: rs.frame) -> float:
            # convert to seconds
            return frame.get_timestamp() * RealSenseCameraSet.TIMESTAMP_FACTOR

        color_extrinsics = self.calibration_strategy[camera_name].get_extrinsics()
        depth_to_color = device.depth_to_color
        assert depth_to_color is not None, "Depth to color extrinsics not found"
        depth_extrinsics = (
            color_extrinsics @ depth_to_color.inverse().pose_matrix() if color_extrinsics is not None else None
        )

        timestamps = []
        for stream in streams:
            if rs.stream.infrared == stream.stream_type():
                frame = frameset.get_infrared_frame(stream.stream_index())
                ir = DataFrame(data=to_numpy(frame), timestamp=to_ts(frame))
            elif rs.stream.color == stream.stream_type():
                frame = frameset.get_color_frame()
                color = DataFrame(
                    data=to_numpy(frame)[:, :, ::-1],
                    timestamp=to_ts(frame),
                    intrinsics=device.color_intrinsics,
                    extrinsics=color_extrinsics,
                )
            elif rs.stream.depth == stream.stream_type():
                frame = frameset.get_depth_frame()
                assert device.depth_scale is not None, "Depth scale not found"
                depth = DataFrame(
                    data=(to_numpy(frame).astype(np.float64) * device.depth_scale * BaseCameraSet.DEPTH_SCALE).astype(
                        np.uint16
                    ),
                    timestamp=to_ts(frame),
                    intrinsics=device.depth_intrinsics,
                    extrinsics=depth_extrinsics,  # type: ignore
                )
            elif rs.stream.accel == stream.stream_type():
                frame = frameset.first(stream.stream_index())
                md = frame.as_motion_frame().get_motion_data()
                accel = DataFrame(data=np.array([md.x, md.y, md.z]), timestamp=to_ts(frame))
            elif rs.stream.gyro == stream.stream_type():
                frame = frameset.first(stream.stream_index())
                md = frame.as_motion_frame().get_motion_data()
                gyro = DataFrame(data=np.array([md.x, md.y, md.z]), timestamp=to_ts(frame))
            else:
                msg = f"Unknown stream type {stream.stream_type()}"
                self._logger.warning(msg)
                continue
            timestamps.append(to_ts(frame))

        assert color is not None, "Color frame not found"
        cf = CameraFrame(
            color=color,
            ir=ir,
            depth=depth,
        )
        imu = IMUFrame(accel=accel, gyro=gyro)
        f = Frame(camera=cf, imu=imu, avg_timestamp=float(np.mean(timestamps)) if len(timestamps) > 0 else None)
        with self._frame_buffer_lock[camera_name]:
            if len(self._frame_buffer[camera_name]) >= self.CALIBRATION_FRAME_SIZE:
                self._frame_buffer[camera_name].pop(0)
            self._frame_buffer[camera_name].append(copy.deepcopy(f))
        return f

    def disable_streams(self):
        self.D400_config.disable_all_streams()

    def close(self):
        for device in self._enabled_devices.values():
            device.pipeline.stop()
        self.disable_streams()
        sleep(5)

    def enable_emitter(self, enable_ir_emitter=True):
        """
        Enable/Disable the emitter of the intel realsense device

        """
        for device in self._enabled_devices.values():
            # Get the active profile and enable the emitter for all the connected devices
            sensor = device.pipeline_profile.get_device().first_depth_sensor()
            if not sensor.supports(rs.option.emitter_enabled):
                continue
            sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
            if enable_ir_emitter:
                sensor.set_option(rs.option.laser_power, self.laser_power)

    def calibrate(self) -> bool:
        for camera_name in self.cameras:
            color_intrinsics = self._enabled_devices[camera_name].color_intrinsics
            assert color_intrinsics is not None, f"Color intrinsics for camera {camera_name} not found"
            if not self.calibration_strategy[camera_name].calibrate(
                intrinsics=color_intrinsics,
                samples=self._frame_buffer[camera_name],
                lock=self._frame_buffer_lock[camera_name],
            ):
                self._logger.warning(f"Calibration of camera {camera_name} failed.")
                return False
        self._logger.info("Calibration successful.")
        return True
