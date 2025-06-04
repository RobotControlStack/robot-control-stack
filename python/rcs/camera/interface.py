import logging
from dataclasses import dataclass
from datetime import datetime
from time import sleep, time
from typing import Any, Protocol

from rcs._core.common import BaseCameraConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SimpleFrameRate:
    def __init__(self, frame_rate: int | float):
        self.t: float | None = None
        self._last_print: float | None = None
        self.frame_rate = frame_rate

    def reset(self):
        self.t = None

    def __call__(self):
        if self.t is None:
            self.t = time()
            self._last_print = self.t
            sleep(1 / self.frame_rate if isinstance(self.frame_rate, int) else self.frame_rate)
            return
        sleep_time = (
            1 / self.frame_rate - (time() - self.t)
            if isinstance(self.frame_rate, int)
            else self.frame_rate - (time() - self.t)
        )
        if sleep_time > 0:
            sleep(sleep_time)
        if self._last_print is None or time() - self._last_print > 10:
            self._last_print = time()
            logger.info(f"FPS: {1 / (time() - self.t)}")

        self.t = time()


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

    def config(self, camera_name: str) -> BaseCameraConfig:
        """Returns the configuration object of the cameras."""

    @property
    def camera_names(self) -> list[str]:
        """Returns a list of the activated human readable names of the cameras."""

    @property
    def name_to_identifier(self) -> dict[str, str]:
        """Dict mapping from human readable name to identifier."""
