import logging
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Literal, Protocol

import numpy as np
from rcs._core.common import BaseCameraConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


@dataclass(kw_only=True)
class DataFrame:
    data: Any
    # timestamp in posix time
    timestamp: float | None = None
    intrinsics: np.ndarray[tuple[Literal[3, 4]], np.dtype[np.float64]] | None = None
    extrinsics: np.ndarray[tuple[Literal[4, 4]], np.dtype[np.float64]] | None = None


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
