import copy
import datetime
import logging
import threading
import typing

import cv2
import numpy as np
from rcs.camera.hw import CalibrationStrategy, DummyCalibrationStrategy, HardwareCamera
from rcs.camera.interface import CameraFrame, DataFrame, Frame

from rcs import common

"""
A generic extension class for handling USB-connected cameras. 
Uses OpenCV to interface with the camera hardware, specifically using cv2.VideoCapture(id).
The ID can be both a single integer passed as a string, i.e. str(0), str(1), or the full /dev/ path, like /dev/video0.

"""


class USBCameraConfig(common.BaseCameraConfig):
    color_intrinsics: np.ndarray[tuple[typing.Literal[3], typing.Literal[4]], np.dtype[np.float64]] | None = None
    distortion_coeffs: np.ndarray[tuple[typing.Literal[5]], np.dtype[np.float64]] | None = None


class USBCameraSet(HardwareCamera):
    def __init__(
        self,
        cameras: dict[str, USBCameraConfig],
        calibration_strategy: dict[str, CalibrationStrategy] | None = None,
    ):
        self.cameras = cameras
        self.CALIBRATION_FRAME_SIZE = 30
        if calibration_strategy is None:
            calibration_strategy = {camera_name: DummyCalibrationStrategy() for camera_name in cameras}
        for cam in self.cameras.values():
            if cam.color_intrinsics is None:
                cam.color_intrinsics = np.zeros((3, 4), dtype=np.float64)  # type: ignore
            if cam.distortion_coeffs is None:
                cam.distortion_coeffs = np.zeros((5,), dtype=np.float64)  # type: ignore
            if cam.resolution_height is None:
                cam.resolution_height = 480
            if cam.resolution_width is None:
                cam.resolution_width = 640
            if cam.frame_rate is None:
                cam.frame_rate = 30
        self.calibration_strategy = calibration_strategy
        self._camera_names = list(self.cameras.keys())
        self._captures: dict[str, cv2.VideoCapture] = {}
        self._logger = logging.getLogger(__name__)
        self._logger.info("USBCamera initialized with cameras: %s", self._camera_names)
        self._logger.info(
            "If the camera streams are not correct, try v4l2-ctl --list-devices to see the available cameras."
        )
        self._frame_buffer_lock: dict[str, threading.Lock] = {}
        self._frame_buffer: dict[str, list] = {}

    def open(self):
        for name, camera in self.cameras.items():
            self._frame_buffer_lock[name] = threading.Lock()
            self._frame_buffer[name] = []
            cap = cv2.VideoCapture(camera.identifier)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera.resolution_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.resolution_height)
            cap.set(cv2.CAP_PROP_FPS, camera.frame_rate)

            if not cap.isOpened():
                msg = f"Could not open camera {name} with id {camera.identifier}"
                raise RuntimeError(msg)
            self._captures[name] = cap

    @property
    def camera_names(self) -> list[str]:
        return self._camera_names

    def poll_frame(self, camera_name: str) -> Frame:
        cap = self._captures[camera_name]
        timestamp = datetime.datetime.now().timestamp()
        ret, color_frame = cap.read()
        if not ret:
            msg = f"Failed to read frame from camera {camera_name}"
            raise RuntimeError(msg)
        with self._frame_buffer_lock[camera_name]:
            if len(self._frame_buffer[camera_name]) >= self.CALIBRATION_FRAME_SIZE:
                self._frame_buffer[camera_name].pop(0)
            self._frame_buffer[camera_name].append(copy.deepcopy(color_frame))
        color = DataFrame(
            data=color_frame,
            timestamp=timestamp,
            intrinsics=self.cameras[camera_name].color_intrinsics,
            extrinsics=self.calibration_strategy[camera_name].get_extrinsics(),
        )
        depth_frame = np.zeros(
            (self.cameras[camera_name].resolution_height, self.cameras[camera_name].resolution_width), dtype=np.uint16
        )
        depth = DataFrame(
            data=depth_frame,
            timestamp=timestamp,
            intrinsics=self.cameras[camera_name].color_intrinsics,
            extrinsics=self.calibration_strategy[camera_name].get_extrinsics(),
        )
        cf = CameraFrame(color=color, depth=depth)
        return Frame(camera=cf, avg_timestamp=timestamp)

    def close(self):
        for cap in self._captures.values():
            cap.release()
        self._captures = {}

    def config(self, camera_name: str) -> USBCameraConfig:
        return self.cameras[camera_name]

    def calibrate(self) -> bool:
        for camera_name in self.cameras:
            color_intrinsics = self.cameras[camera_name].color_intrinsics
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
