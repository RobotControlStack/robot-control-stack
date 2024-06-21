import logging
from datetime import datetime

import mujoco as mj
import numpy as np
from rcsss._core.sim import FrameSet as _FrameSet
from rcsss._core.sim import SimCameraConfig as _SimCameraConfig
from rcsss._core.sim import SimCameraSet as _SimCameraSet
from rcsss.camera.interface import (
    BaseCameraConfig,
    CameraFrame,
    DataFrame,
    Frame,
    FrameSet,
)


class SimCameraConfig(BaseCameraConfig):
    camera2id: dict[str, str] = {}


class SimCameraSet(_SimCameraSet):
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet
    """

    def __init__(self, sim, cfg: SimCameraConfig):
        self._logger = logging.getLogger(__name__)
        self._cfg = cfg
        cpp_cfg = _SimCameraConfig()
        cpp_cfg.camera2id = cfg.camera2id
        cpp_cfg.frame_rate = cfg.frame_rate
        cpp_cfg.resolution_width = cfg.resolution_width
        cpp_cfg.resolution_height = cfg.resolution_height

        super().__init__(sim, cpp_cfg)

    def get_latest_frames(self) -> FrameSet | None:
        """Should return the latest frame from the camera with the given name."""
        return self._cpp_to_python_frames(super().get_latest_frames())

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Should return the frame from the camera with the given name and closest to the given timestamp."""
        return self._cpp_to_python_frames(super().get_timestamp_frames(ts.timestamp()))

    def _cpp_to_python_frames(self, cpp_frameset: _FrameSet | None) -> FrameSet | None:
        if cpp_frameset is None:
            return None
        frames: dict[str, Frame] = {}
        for frame_name, cpp_frame in cpp_frameset.color_frames.items():
            frame_name: str
            cpp_frame: np.ndarray
            cameraframe = CameraFrame(color=DataFrame(data=cpp_frame, timestamp=cpp_frameset.timestamp))
            frame = Frame(camera=cameraframe, avg_timestamp=cpp_frameset.timestamp)
            frames[frame_name] = frame
        return FrameSet(frames=frames, avg_timestamp=cpp_frameset.timestamp)

    @property
    def config(self) -> SimCameraConfig:
        return self._cfg

    @property
    def camera_names(self) -> list[str]:
        """Returns a list of the activated human readable names of the cameras."""
        return list(self._cfg.camera2id.keys())
