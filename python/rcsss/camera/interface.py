from abc import ABC, abstractmethod
from time import sleep
from dataclasses import dataclass
from datetime import datetime
import logging
from pathlib import Path
import pickle
from typing import Any

import numpy as np
from pydantic import BaseModel
import threading




class BaseCameraConfig(BaseModel):
    frame_rate: int = 15  # fps
    warm_up_disposal_frames: int = 30  # frames
    record_path: str = "camera_frames"

@dataclass(kw_only=True)
class DataFrame():
    data: Any
    # timestamp in posix time
    timestamp: float | None = None

@dataclass(kw_only=True)
class CameraFrame():
    color: DataFrame | None = None
    ir: DataFrame | None = None
    depth: DataFrame | None = None
    temperature: float | None = None


@dataclass(kw_only=True)
class IMUFrame():
    accel: DataFrame | None = None
    gyro: DataFrame | None = None
    temperature: float | None = None


@dataclass(kw_only=True)
class Frame():
    camera: CameraFrame
    imu: IMUFrame | None
    avg_timestamp: float | None

@dataclass(kw_only=True)
class FrameSet():
    frames: dict[str, Frame]
    avg_timestamp: float | None


class BaseCameraSet():
    """This base class should have the ability to poll in a separate thread for all cameras and store them in a buffer.
    """

    def __init__(self):
        self._buffer: list[FrameSet] = []
        self._buffer_lock = threading.Lock()
        self.running = False
        self._thread: threading.Thread | None = None
        self._logger = logging.getLogger(__name__)


    def get_latest_frames(self) -> FrameSet | None:
        """Should return the latest frame from the camera with the given name.
        """
        with self._buffer_lock:
            return self._buffer[-1] if len(self._buffer) > 0 else None

        
    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Should return the frame from the camera with the given name and closest to the given timestamp.
        """
        # iterate through the buffer and find the closest timestamp
        with self._buffer_lock:
            for frame_set in reversed(self._buffer):
                if frame_set['avg_timestamp'] <= ts.timestamp():
                    return frame_set
            return None

    def stop(self):
        """Stops the polling of the cameras.
        """
        self.running = False
        self._thread.join()
        self._save_frames()

    def start(self, warm_up: bool = True):
        """Should start the polling of the cameras.
        """
        self.running = True
        self._thread = threading.Thread(target=self.polling_thread, args=(warm_up,))
        self._thread.start()

    def warm_up(self):
        for _ in range(self.config.warm_up_disposal_frames):
            for camera_name in self.camera_names:
                self._poll_frame(camera_name)
            sleep(1/self.config.frame_rate)


    def polling_thread(self, warm_up: bool = True):
        if warm_up:
            self.warm_up()
        while self.running:
            frame_set = self.poll_frame_set()
            with self._buffer_lock:
                self._buffer.append(frame_set)
            sleep(1/self.config.frame_rate)

    def poll_frame_set(self) -> FrameSet:
        """Gather frames over all available cameras.
        """
        frames = {}
        for camera_name in self.camera_names:
            frame = self._poll_frame(camera_name)
            frames[camera_name] = frame
        return FrameSet(frames=frames, avg_timestamp=np.mean([frame.avg_timestamp for frame in frames.values()]))

    def _save_frames(self):
        """Saves all frames from the buffer in python pickle format and clears the buffer.
        """
        with self._buffer_lock:
            pickle.dump(self._buffer, open(Path(self.config.record_path) / f"frames_{datetime.now().timestamp()}.pk", "wb"))
            self._logger.debug(f"Saved {len(self._buffer)} frames.")
            self._buffer = []


    @property
    @abstractmethod
    def config(self) -> BaseCameraConfig:
        """Should return the configuration object of the cameras.
        """
        

    @abstractmethod
    def _poll_frame(self, camera_name: str) -> Frame:
        """Should return the latest frame from the camera with the given name.

        This method should be thread safe.
        """

    @property
    @abstractmethod
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras.
        """

