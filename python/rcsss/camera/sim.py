import logging
from datetime import datetime

from rcsss.camera.interface import BaseCameraConfig, FrameSet


class SimCameraConfig(BaseCameraConfig):
    camera2id: dict[str, str] = {}


# TODO inherit
class SimCameraSet:
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet
    """

    def __init__(self, camera_set, cfg: SimCameraConfig):
        self._cpp_camera_set = camera_set
        self._logger = logging.getLogger(__name__)
        self._cfg = cfg

    def buffer_size(self) -> int:
        return self._cpp_camera_set.buffer_size()

    def get_latest_frames(self) -> FrameSet | None:
        """Should return the latest frame from the camera with the given name."""
        return self._cpp_to_python_frames(self._cpp_camera_set.get_latest_frames())

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Should return the frame from the camera with the given name and closest to the given timestamp."""
        return self._cpp_to_python_frames(self._cpp_camera_set.get_timestamp_frames(ts))

    def _cpp_to_python_frames(self, cpp_frames):
        # TODO: convert cpp_frames to python FrameSet
        frames = {}
        avg_timestamp = None
        return FrameSet(frames=frames, avg_timestamp=avg_timestamp)

    def clear_buffer(self):
        """Deletes all frames from the buffer."""
        self._cpp_camera_set.clear_buffer()

    @property
    def config(self) -> SimCameraConfig:
        return self._cfg

    @property
    def camera_names(self) -> list[str]:
        """Returns a list of the activated human readable names of the cameras."""
        return list(self._cfg.camera2id.keys())
