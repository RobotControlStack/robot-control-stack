from dataclasses import dataclass
from datetime import datetime
from time import time, sleep
from typing import Any, Protocol

from pydantic import BaseModel, Field

class SimpleFrameRate:
    def __init__(self):
        self.t = None

    def reset(self):
        self.t = None

    def __call__(self, frame_rate: int):
        if self.t is None:
            self.t = time()
            sleep(1 / frame_rate)
            return
        sleep_time = 1 / frame_rate - (time() - self.t)
        if sleep_time > 0:
            sleep(sleep_time)
        self.t = time()


class BaseCameraConfig(BaseModel):
    identifier: str


class BaseCameraSetConfig(BaseModel):
    cameras: dict = Field(default={})
    resolution_width: int = 1280  # pixels
    resolution_height: int = 720  # pixels
    frame_rate: int = 15  # Hz

    @property
    def name_to_identifier(self):
        return {key: camera.identifier for key, camera in self.cameras.items()}


@dataclass(kw_only=True)
class DataFrame:
    data: Any
    # timestamp in posix time
    timestamp: float | None = None


@dataclass(kw_only=True)
class CameraFrame:
    color: DataFrame
    ir: DataFrame | None = None
    depth: DataFrame | None = None
    temperature: float | None = None


@dataclass(kw_only=True)
class IMUFrame:
    accel: DataFrame | None = None
    gyro: DataFrame | None = None
    temperature: float | None = None


@dataclass(kw_only=True)
class Frame:
    camera: CameraFrame
    imu: IMUFrame | None = None
    avg_timestamp: float | None = None


@dataclass(kw_only=True)
class FrameSet:
    frames: dict[str, Frame]
    avg_timestamp: float | None


class BaseCameraSet(Protocol):
    """Interface for a set of cameras for sim and hardware"""

    def buffer_size(self) -> int:
        """Returns size of the internal buffer."""

    def get_latest_frames(self) -> FrameSet | None:
        """Returns the latest frame from the camera with the given name."""

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Returns the frame from the camera with the given name and closest to the given timestamp."""

    def clear_buffer(self):
        """Deletes all frames from the buffer."""

    def close(self):
        """Stops any running threads e.g. for exitting."""

    @property
    def config(self) -> BaseCameraSetConfig:
        """Return the configuration object of the cameras."""

    @property
    def camera_names(self) -> list[str]:
        """Returns a list of the activated human readable names of the cameras."""

    @property
    def name_to_identifier(self) -> dict[str, str]:
        """Dict mapping from human readable name to identifier."""
