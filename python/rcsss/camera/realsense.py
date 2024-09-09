from dataclasses import dataclass

import numpy as np
import pyrealsense2 as rs
from rcsss.camera.hw import BaseHardwareCameraSet, HWCameraSetConfig
from rcsss.camera.interface import CameraFrame, DataFrame, Frame, IMUFrame

# class RealSenseConfig(BaseCameraConfig):
# dict with readable name and serial number


class RealSenseSetConfig(HWCameraSetConfig):
    # dict with readable name and serial number
    # devices_to_enable: dict[str, str] = {}
    # cameras: dict[str, RealSenseConfig] = []
    enable_ir_emitter: bool = False
    enable_ir: bool = False
    laser_power: int = 330
    enable_imu: bool = False


@dataclass
class RealSenseDeviceInfo:
    product_line: str
    serial: str


@dataclass
class RealSenseDevicePipeline:
    pipeline: rs.pipeline
    pipeline_profile: rs.pipeline_profile
    camera: RealSenseDeviceInfo


# TODO(juelg): look at frame queue
class RealSenseCameraSet(BaseHardwareCameraSet):
    TIMESTAMP_FACTOR = 1e-3

    def __init__(self, cfg: RealSenseSetConfig) -> None:
        self._cfg = cfg
        super().__init__()
        self.D400_config = rs.config()

        self.D400_config.enable_stream(
            rs.stream.depth,
            self._cfg.resolution_width,
            self._cfg.resolution_height,
            rs.format.z16,
            self._cfg.frame_rate,
        )
        self.D400_config.enable_stream(
            rs.stream.color,
            self._cfg.resolution_width,
            self._cfg.resolution_height,
            rs.format.bgr8,
            self._cfg.frame_rate,
        )
        if self._cfg.enable_ir:
            self.D400_config.enable_stream(
                rs.stream.infrared,
                1,
                self._cfg.resolution_width,
                self._cfg.resolution_height,
                rs.format.y8,
                self._cfg.frame_rate,
            )
        if self._cfg.enable_imu:
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

        self._context = rs.context()
        self._available_devices: dict[str, RealSenseDeviceInfo] = {}
        self.update_available_devices()
        self._enabled_devices: dict[str, RealSenseDevicePipeline] = {}  # serial numbers of te enabled devices
        self.enable_devices(self.name_to_identifier, self._cfg.enable_ir_emitter)

    def update_available_devices(self):
        self._available_devices = self.enumerate_connected_devices(self._context)

    def enable_all_devices(self, enable_ir_emitter: bool = False):
        """
        Enable all the Intel RealSense Devices which are connected to the PC

        """
        self._logger.debug("%i devices have been found", len(self._available_devices))

        for device_info in self._available_devices.values():
            self.enable_device(device_info, enable_ir_emitter)

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
            self.enable_device(self._available_devices[device_serial], enable_ir_emitter)

    def enable_device(self, device_info: RealSenseDeviceInfo, enable_ir_emitter: bool = False):
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
            sensor.set_option(rs.option.laser_power, self._cfg.laser_power)
        self._enabled_devices[device_info.serial] = RealSenseDevicePipeline(pipeline, pipeline_profile, device_info)
        self._logger.debug("Enabled device %s (%s)", device_info.serial, device_info.product_line)

    @property
    def config(self) -> RealSenseSetConfig:
        return self._cfg

    @config.setter
    def config(self, cfg: RealSenseSetConfig) -> None:
        self._cfg = cfg

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

    def _poll_frame(self, camera_name: str) -> Frame:
        # TODO(juelg): polling should be performed in a recorder thread
        # (anyway needed to record trajectory data to disk)
        # and this method should just access the latest frame from the recorder
        # Perhaps there even has to be a separate recorder thread for each camera

        # TODO(juelg): what is a "pose" (in frame) and how can we use it
        # TODO(juelg): decide whether to use the poll method and to wait if devices get ready
        assert camera_name in self.name_to_identifier, f"Camera {camera_name} not found in the enabled devices"
        serial = self.name_to_identifier[camera_name]
        device = self._enabled_devices[serial]

        streams = device.pipeline_profile.get_streams()
        frameset = device.pipeline.wait_for_frames()
        # frameset = device.pipeline.poll_for_frames()

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

        timestamps = []
        for stream in streams:
            if rs.stream.infrared == stream.stream_type():
                frame = frameset.get_infrared_frame(stream.stream_index())
                ir = DataFrame(data=to_numpy(frame), timestamp=to_ts(frame))
            elif rs.stream.color == stream.stream_type():
                frame = frameset.get_color_frame()
                color = DataFrame(data=to_numpy(frame)[:, :, ::-1], timestamp=to_ts(frame))
            elif rs.stream.depth == stream.stream_type():
                frame = frameset.get_depth_frame()
                depth = DataFrame(data=to_numpy(frame), timestamp=to_ts(frame))
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
        cf = CameraFrame(color=color, ir=ir, depth=depth)
        imu = IMUFrame(accel=accel, gyro=gyro)
        return Frame(camera=cf, imu=imu, avg_timestamp=float(np.mean(timestamps)) if len(timestamps) > 0 else None)

    def get_depth_shape(self):
        """
        Retruns width and height of the depth stream for one arbitrary device

        Returns:
        -----------
        width : int
        height: int
        """
        width = -1
        height = -1
        for device in self._enabled_devices.values():
            for stream in device.pipeline_profile.get_streams():
                if rs.stream.depth == stream.stream_type():
                    width = stream.as_video_stream_profile().width()
                    height = stream.as_video_stream_profile().height()
        return width, height

    def get_device_intrinsics(
        self, frames: dict[str, dict[rs.stream, rs.frame]]
    ) -> dict[str, dict[rs.stream, rs.intrinsics]]:
        """
        Get the intrinsics of the imager using its frame delivered by the realsense device

        Parameters:
        -----------
        frames : rs::frame
                 The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed

        Return:
        -----------
        device_intrinsics : dict
        keys  : serial
                Serial number of the device
        values: [key]
                Intrinsics of the corresponding device
        """
        device_intrinsics: dict[str, dict[rs.stream, rs.intrinsics]] = {}
        for device_name, frameset in frames.items():
            device_intrinsics[device_name] = {}
            for key, value in frameset.items():
                device_intrinsics[device_name][key] = value.get_profile().as_video_stream_profile().get_intrinsics()
        return device_intrinsics

    def get_depth_to_color_extrinsics(self, frames):
        """
        Get the extrinsics between the depth imager 1 and the color imager using its frame delivered by the realsense device

        Parameters:
        -----------
        frames : rs::frame
                 The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed

        Return:
        -----------
        device_intrinsics : dict
        keys  : serial
                Serial number of the device
        values: [key]
                Extrinsics of the corresponding device
        """
        device_extrinsics = {}
        for dev_info, frameset in frames.items():
            serial = dev_info[0]
            device_extrinsics[serial] = (
                frameset[rs.stream.depth]
                .get_profile()
                .as_video_stream_profile()
                .get_extrinsics_to(frameset[rs.stream.color].get_profile())
            )
        return device_extrinsics

    def disable_streams(self):
        self.D400_config.disable_all_streams()

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
                sensor.set_option(rs.option.laser_power, self._cfg.laser_power)

    def load_settings_json(self, path_to_settings_file):
        """
        Load the settings stored in the JSON file

        """

        with open(path_to_settings_file, "r") as file:
            json_text = file.read().strip()

        for device in self._enabled_devices.values():
            if device.camera.product_line != "D400":
                continue
            # Get the active profile and load the json file which contains settings readable by the realsense
            dev = device.pipeline_profile.get_device()
            advanced_mode = rs.rs400_advanced_mode(dev)
            advanced_mode.load_json(json_text)
