from asyncio import sleep
from dataclasses import dataclass
import numpy as np
from rcsss.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, BaseCameraConfig, IMUFrame
import pyrealsense2 as rs


import logging

logger = logging.getLogger(__name__)


class RealSenseConfig(BaseCameraConfig):
    resolution_width: int = 1280  # pixels
    resolution_height: int = 720  # pixels
    frame_rate: int = 15  # fps
    dispose_frames_for_stablisation: int = 30  # frames
    devices_to_enable: dict[str, str] = {}  # dict with readable name and serial number
    enable_ir_emitter: bool = False
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


class RealSenseCameraSet(BaseCameraSet):
    def __init__(self, cfg: RealSenseConfig) -> None:
        self._cfg = cfg
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
        if self._cfg.enable_ir_emitter:
            self.D400_config.enable_stream(
                rs.stream.infrared,
                1,
                self._cfg.resolution_width,
                self._cfg.resolution_height,
                rs.format.y8,
                self._cfg.frame_rate,
            )
        if self._cfg.enable_imu:
            self.D400_config.enable_stream(
                rs.stream.accel,
                rs.format.motion_xyz32f,
                self._cfg.frame_rate,
            )
            self.D400_config.enable_stream(
                rs.stream.gyro,
                rs.format.motion_xyz32f,
                self._cfg.frame_rate,
            )

        self._context = rs.context()
        self._available_devices: dict[str, RealSenseDeviceInfo] = {}
        self.update_available_devices()
        self._enabled_devices: dict[str, RealSenseDevicePipeline] = {}  # serial numbers of te enabled devices

        self.enable_devices(self._cfg.devices_to_enable, self._cfg.enable_ir_emitter)

        # Allow some frames for the auto-exposure controller to stablise
        # for _ in range(self._cfg.dispose_frames_for_stablisation):
        #     self.poll_frames()

    def update_available_devices(self):
        self._available_devices = self.enumerate_connected_devices(self._context)

    def enable_all_devices(self, enable_ir_emitter: bool = False):
        """
        Enable all the Intel RealSense Devices which are connected to the PC

        """
        print(str(len(self._available_devices)) + " devices have been found")

        for device_info in self._available_devices:
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
            raise RuntimeError(f"unknown product line {device_info.product_line}")

        # Set the acquisition parameters
        sensor = pipeline_profile.get_device().first_depth_sensor()
        if sensor.supports(rs.option.emitter_enabled):
            sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
            sensor.set_option(rs.option.laser_power, self._cfg.laser_power)
        self._enabled_devices[device_info.serial] = RealSenseDevicePipeline(pipeline, pipeline_profile, device_info)

    @property
    def config(self) -> RealSenseConfig:
        return self._cfg

    @config.setter
    def config(self, cfg: RealSenseConfig) -> None:
        self._cfg = cfg

    def get_frame_timestamp(self, camera_name: str, ts: str) -> Frame:
        pass

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
                # device_info = (serial, product_line)  # (serial_number, product_line)
                connect_device[serial] = device_info
        return connect_device

    # def poll_frames(self) -> dict:
    #     """
    #     Poll for frames from the enabled Intel RealSense devices. This will return at least one frame from each device.
    #     If temporal post processing is enabled, the depth stream is averaged over a certain amount of frames

    #     Parameters:
    #     -----------
    #     """
    #     frames = {}
    #     while len(frames) < len(self._enabled_devices.items()):
    #         for serial, device in self._enabled_devices.items():
    #             streams = device.pipeline_profile.get_streams()
    #             frameset = device.pipeline.poll_for_frames()  # frameset will be a pyrealsense2.composite_frame object
    #             if frameset.size() == len(streams):
    #                 dev_info = (serial, device.product_line)
    #                 frames[dev_info] = {}
    #                 for stream in streams:
    #                     if rs.stream.infrared == stream.stream_type():
    #                         frame = frameset.get_infrared_frame(stream.stream_index())
    #                         key_ = (stream.stream_type(), stream.stream_index())
    #                     else:
    #                         frame = frameset.first_or_default(stream.stream_type())
    #                         key_ = stream.stream_type()
    #                     frames[dev_info][key_] = frame
    #     return frames

    def get_frame_latest(self, camera_name: str) -> Frame:
        # TODO(juelg): polling should be performed in a recorder thread
        # (anyway needed to record trajectory data to disk)
        # and this method should just access the latest frame from the recorder
        # Perhaps there even has to be a separate recorder thread for each camera

        # TODO(juelg): what is a "pose" (in frame) and how can we use it

        assert camera_name in self._cfg.devices_to_enable, f"Camera {camera_name} not found in the enabled devices"
        serial = self._cfg.devices_to_enable[camera_name]
        device = self._enabled_devices[serial]

        streams = device.pipeline_profile.get_streams()
        frameset = device.pipeline.wait_for_frames()

        color: DataFrame | None = None
        ir: DataFrame | None = None
        depth: DataFrame | None = None
        accel: DataFrame | None = None
        gyro: DataFrame | None = None

        def to_numpy(frame: rs.frame) -> np.ndarray:
            return np.asanyarray(frame.get_data())

        timestamps = []
        for stream in streams:
            if rs.stream.infrared == stream.stream_type():
                frame = frameset.get_infrared_frame(stream.stream_index())
                ir = DataFrame(to_numpy(frame), frame.get_timestamp())
            elif rs.stream.color == stream.stream_type():
                frame = frameset.get_color_frame()
                color = DataFrame(to_numpy(frame), frame.get_timestamp())
            elif rs.stream.depth == stream.stream_type():
                frame = frameset.get_depth_frame()
                depth = DataFrame(to_numpy(frame), frame.get_timestamp())
            elif rs.stream.accel == stream.stream_type():
                frame = frameset.first(stream.stream_index())
                md = frame.as_motion_frame().get_motion_data()
                accel = DataFrame(np.array([md.x, md.y, md.z]), frame.get_timestamp())
            elif rs.stream.gyro == stream.stream_type():
                frame = frameset.first(stream.stream_index())
                md = frame.as_motion_frame().get_motion_data()
                gyro = DataFrame(np.array([md.x, md.y, md.z]), frame.get_timestamp())
            else:
                logger.warning(f"Unknown stream type {stream.stream_type()}")
                continue
            timestamps.append(frame.get_timestamp())

        assert color is not None, "Color frame not found"

        cf = CameraFrame(color=color, ir=ir, depth=depth)
        imu = IMUFrame(accel=accel, gyro=gyro)
        return Frame(camera=cf, imu=imu, avg_timestamp=np.mean(timestamps))

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

    def get_device_intrinsics(self, frames):
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
        device_intrinsics = {}
        for dev_info, frameset in frames.items():
            serial = dev_info[0]
            device_intrinsics[serial] = {}
            for key, value in frameset.items():
                device_intrinsics[serial][key] = value.get_profile().as_video_stream_profile().get_intrinsics()
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

        for device_serial, device in self._enabled_devices.items():
            if device.camera.product_line != "D400":
                continue
            # Get the active profile and load the json file which contains settings readable by the realsense
            device = device.pipeline_profile.get_device()
            advanced_mode = rs.rs400_advanced_mode(device)
            advanced_mode.load_json(json_text)

# TODO: test the realsense camera
# - program that reads out the serial number

if __name__ == "__main__":
    pass
