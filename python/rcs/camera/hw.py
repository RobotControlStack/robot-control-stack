import logging
import threading
import typing
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from time import sleep

import cv2
import numpy as np
from rcs._core.common import BaseCameraConfig
from rcs.camera.interface import BaseCameraSet, Frame, FrameSet
from rcs.utils import SimpleFrameRate


class HardwareCamera(typing.Protocol):
    """Implementation of a hardware camera potentially a set of cameras of the same kind."""

    def open(self):
        """Should open the camera and prepare it for polling."""

    def close(self):
        """Should close the camera and release all resources."""

    def config(self, camera_name: str) -> BaseCameraConfig:
        """Should return the configuration object of the cameras."""

    def poll_frame(self, camera_name: str) -> Frame:
        """Should return the latest frame from the camera with the given name.

        This method should be thread safe.
        """

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""


class HardwareCameraSet(BaseCameraSet):
    """This base class polls in a separate thread for all cameras and stores them in a buffer.

    Cameras can consist of multiple cameras, e.g. RealSense cameras.
    """

    def __init__(
        self, cameras: Sequence[HardwareCamera], warm_up_disposal_frames: int = 30, max_buffer_frames: int = 1000
    ):
        self.cameras = cameras
        self.camera_dict, self._camera_names = self._cameras_util()
        self.frame_rate = self._frames_rate()
        self.rate_limiter = SimpleFrameRate(self.frame_rate)

        self.warm_up_disposal_frames = warm_up_disposal_frames
        self.max_buffer_frames = max_buffer_frames
        self._buffer: list[FrameSet | None] = [None for _ in range(self.max_buffer_frames)]
        self._buffer_lock = threading.Lock()
        self.running = False
        self._thread: threading.Thread | None = None
        self._logger = logging.getLogger(__name__)
        self._next_ring_index = 0
        self._buffer_len = 0
        self.writer: dict[str, cv2.VideoWriter] = {}

    @property
    def camera_names(self) -> list[str]:
        """Returns the names of the cameras in this set."""
        return self._camera_names

    @property
    def name_to_identifier(self) -> dict[str, str]:
        """Returns a dictionary mapping the camera names to their identifiers."""
        name_to_id: dict[str, str] = {}
        for camera in self.cameras:
            for name in camera.camera_names:
                name_to_id[name] = camera.config(name).identifier
        return name_to_id

    def _frames_rate(self) -> int:
        """Checks if all cameras have the same frame rate."""
        frame_rates = {camera.config(name).frame_rate for camera in self.cameras for name in camera.camera_names}
        if len(frame_rates) > 1:
            msg = "All cameras must have the same frame rate. Different frame rates are not supported."
            raise ValueError(msg)
        if len(frame_rates) == 0:
            self._logger.warning("No camera found, empty polling with 1 fps.")
            return 1
        return next(iter(frame_rates))

    def _cameras_util(self) -> tuple[dict[str, HardwareCamera], list[str]]:
        """Utility function to create a dictionary of cameras and a list of camera names."""
        camera_dict: dict[str, HardwareCamera] = {}
        camera_names: list[str] = []
        for camera in self.cameras:
            camera_names.extend(camera.camera_names)
            for name in camera.camera_names:
                assert name not in camera_dict, f"Camera name {name} not unique."
                camera_dict[name] = camera
        return camera_dict, camera_names

    def buffer_size(self) -> int:
        return len(self._buffer) - self._buffer.count(None)

    def wait_for_frames(self, timeout: float = 10.0):
        while self.buffer_size() == 0:
            sleep(0.1)
            timeout -= 0.1
            if timeout < 0:
                self._logger.error("Timeout waiting for frames")
                raise

    def get_latest_frames(self) -> FrameSet | None:
        """Should return the latest frame from the camera with the given name."""
        with self._buffer_lock:
            return self._buffer[self._next_ring_index - 1] if self._buffer_len > 0 else None

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Should return the frame from the camera with the given name and closest to the given timestamp."""
        # iterate through the buffer and find the closest timestamp
        with self._buffer_lock:
            for i in range(self._buffer_len):
                idx = (self._next_ring_index - i - 1) % self.max_buffer_frames  # iterate backwards
                assert self._buffer[idx] is not None
                item: FrameSet = typing.cast(FrameSet, self._buffer[idx])
                assert item.avg_timestamp is not None
                if item.avg_timestamp <= ts.timestamp():
                    return self._buffer[idx]
            return None

    def stop(self):
        """Stops the polling of the cameras."""
        self.running = False
        assert self._thread is not None
        self._thread.join()
        self._thread = None

    def close(self):
        if self.running and self._thread is not None:
            self.stop()
        for camera in self.cameras:
            camera.close()
        self.stop_video()

    def start(self, warm_up: bool = True):
        """Should start the polling of the cameras."""
        if self.running:
            self._logger.warning("Camera thread already running!")
            return
        self.running = True
        self._thread = threading.Thread(target=self.polling_thread, args=(warm_up,))
        self._thread.start()

    def record_video(self, path: Path, str_id: str):
        if self.recording_ongoing():
            return
        for camera in self.camera_names:
            self.writer[camera] = cv2.VideoWriter(
                str(path / f"episode_{str_id}_{camera}.mp4"),
                # migh require to install ffmpeg
                cv2.VideoWriter_fourcc(*"mp4v"),  # type: ignore
                self.frame_rate,
                (self.config(camera).resolution_width, self.config(camera).resolution_height),
            )

    def recording_ongoing(self) -> bool:
        with self._buffer_lock:
            return len(self.writer) > 0

    def stop_video(self):
        if len(self.writer) > 0:
            with self._buffer_lock:
                for camera in self.camera_names:
                    self.writer[camera].release()
                self.writer = {}

    def warm_up(self):
        for _ in range(self.warm_up_disposal_frames):
            for camera_name in self.camera_names:
                self.poll_frame(camera_name)
            self.rate_limiter()

    def polling_thread(self, warm_up: bool = True):
        for camera in self.cameras:
            camera.open()
        if warm_up:
            self.warm_up()
        while self.running:
            frame_set = self.poll_frame_set()
            # buffering
            with self._buffer_lock:
                self._buffer[self._next_ring_index] = frame_set
                self._next_ring_index = (self._next_ring_index + 1) % self.max_buffer_frames
                self._buffer_len = max(self._buffer_len + 1, self.max_buffer_frames)
            # video recording
            for camera_key, writer in self.writer.items():
                if frame_set is not None:
                    writer.write(frame_set.frames[camera_key].camera.color.data[:, :, ::-1])
            self.rate_limiter()

    def poll_frame_set(self) -> FrameSet:
        """Gather frames over all available cameras."""
        frames: dict[str, Frame] = {}
        for camera_name in self.camera_names:
            # callback
            frame = self.poll_frame(camera_name)
            frames[camera_name] = frame
        # filter none
        timestamps: list[float] = [frame.avg_timestamp for frame in frames.values() if frame.avg_timestamp is not None]
        return FrameSet(frames=frames, avg_timestamp=float(np.mean(timestamps)) if len(timestamps) > 0 else None)

    def clear_buffer(self):
        """Deletes all frames from the buffer."""
        with self._buffer_lock:
            self._buffer = [None for _ in range(self.max_buffer_frames)]
            self._next_ring_index = 0
            self._buffer_len = 0
        self.wait_for_frames()

    def config(self, camera_name: str) -> BaseCameraConfig:
        """Returns the configuration object of the cameras."""
        return self.camera_dict[camera_name].config(camera_name)

    def poll_frame(self, camera_name: str) -> Frame:
        return self.camera_dict[camera_name].poll_frame(camera_name)
