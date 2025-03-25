import logging
import threading
import typing
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from time import sleep

import cv2
import numpy as np
from pydantic import Field
from rcsss.camera.interface import (
    BaseCameraConfig,
    BaseCameraSetConfig,
    Frame,
    FrameSet,
    SimpleFrameRate,
)


class HWCameraSetConfig(BaseCameraSetConfig):
    cameras: dict[str, BaseCameraConfig] = Field(default={})
    warm_up_disposal_frames: int = 30  # frames
    record_path: str = "camera_frames"
    max_buffer_frames: int = 1000


# TODO(juelg): refactor camera thread into their own class, to avoid a base hardware camera set class
# TODO(juelg): add video recording
class BaseHardwareCameraSet(ABC):
    """This base class should have the ability to poll in a separate thread for all cameras and store them in a buffer.
    Implements BaseCameraSet
    """

    def __init__(self):
        self._buffer: list[FrameSet | None] = [None for _ in range(self.config.max_buffer_frames)]
        self._buffer_lock = threading.Lock()
        self.running = False
        self._thread: threading.Thread | None = None
        self._logger = logging.getLogger(__name__)
        self._next_ring_index = 0
        self._buffer_len = 0
        self.writer: dict[str, cv2.VideoWriter] = {}
        self.rate = SimpleFrameRate()

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
                idx = (self._next_ring_index - i - 1) % self.config.max_buffer_frames  # iterate backwards
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
        self.stop_video()

    def start(self, warm_up: bool = True):
        """Should start the polling of the cameras."""
        if self.running:
            self._logger.warning("Camera thread already running!")
            return
        self.running = True
        self._thread = threading.Thread(target=self.polling_thread, args=(warm_up,))
        self._thread.start()

    def record_video(self, path: Path, episode: int):
        for camera in self.camera_names:
            self.writer[camera] = cv2.VideoWriter(
                str(path / f"episode_{episode}_{camera}.mp4"),
                # migh require to install ffmpeg
                cv2.VideoWriter_fourcc(*"mp4v"),  # type: ignore
                self.config.frame_rate,
                (self.config.resolution_width, self.config.resolution_height),
            )

    def stop_video(self):
        if len(self.writer) > 0:
            for camera_key, writer in self.writer.items():
                for i in range(self._next_ring_index):
                    frameset = self._buffer[i]
                    assert frameset is not None
                    # rgb to bgr as expected by opencv
                    writer.write(frameset.frames[camera_key].camera.color.data[:, :, ::-1])
            for camera in self.camera_names:
                self.writer[camera].release()
            self.writer = {}

    def warm_up(self):
        for _ in range(self.config.warm_up_disposal_frames):
            for camera_name in self.camera_names:
                self._poll_frame(camera_name)
            self.rate(self.config.frame_rate)

    def polling_thread(self, warm_up: bool = True):
        if warm_up:
            self.warm_up()
        while self.running:
            frame_set = self.poll_frame_set()
            with self._buffer_lock:
                self._buffer[self._next_ring_index] = frame_set
                # copy the buffer to the record path
                for camera_key, writer in self.writer.items():
                    frameset = self._buffer[self._next_ring_index]
                    assert frameset is not None
                    writer.write(frameset.frames[camera_key].camera.color.data[:, :, ::-1])
                self._next_ring_index = (self._next_ring_index + 1) % self.config.max_buffer_frames
                self._buffer_len = max(self._buffer_len + 1, self.config.max_buffer_frames)
            self.rate(self.config.frame_rate)

    def poll_frame_set(self) -> FrameSet:
        """Gather frames over all available cameras."""
        frames: dict[str, Frame] = {}
        for camera_name in self.camera_names:
            frame = self._poll_frame(camera_name)
            frames[camera_name] = frame
        # filter none
        timestamps: list[float] = [frame.avg_timestamp for frame in frames.values() if frame.avg_timestamp is not None]
        return FrameSet(frames=frames, avg_timestamp=float(np.mean(timestamps)) if len(timestamps) > 0 else None)

    def clear_buffer(self):
        """Deletes all frames from the buffer."""
        with self._buffer_lock:
            self._buffer = [None for _ in range(self.config.max_buffer_frames)]
            self._next_ring_index = 0
            self._buffer_len = 0

    @property
    @abstractmethod
    def config(self) -> HWCameraSetConfig:
        """Should return the configuration object of the cameras."""

    @abstractmethod
    def _poll_frame(self, camera_name: str) -> Frame:
        """Should return the latest frame from the camera with the given name.

        This method should be thread safe.
        """

    @property
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras."""
        return list(self.config.cameras)

    @property
    def name_to_identifier(self) -> dict[str, str]:
        # return {key: camera.identifier for key, camera in self._cfg.cameras.items()}
        return self.config.name_to_identifier
