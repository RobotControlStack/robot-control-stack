import logging
from datetime import datetime

import numpy as np
import rcsss
from pydantic import Field
from rcsss._core.sim import CameraType
from rcsss._core.sim import FrameSet as _FrameSet
from rcsss._core.sim import SimCameraConfig as _SimCameraConfig
from rcsss._core.sim import SimCameraSet as _SimCameraSet
from rcsss._core.sim import SimCameraSetConfig as _SimCameraSetConfig
from rcsss.camera.interface import (
    BaseCameraConfig,
    BaseCameraSetConfig,
    CameraFrame,
    DataFrame,
    Frame,
    FrameSet,
)


class SimCameraConfig(BaseCameraConfig):
    type: int  # CamBaseCameraConfigeraType


class SimCameraSetConfig(BaseCameraSetConfig):
    cameras: dict[str, SimCameraConfig] = Field(default={})
    max_buffer_frames: int = 1000
    physical_units: bool = False


class SimCameraSet(_SimCameraSet):
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet
    """

    def __init__(self, sim: rcsss.sim.Sim, cfg: SimCameraSetConfig):
        self._logger = logging.getLogger(__name__)
        self._cfg = cfg
        cameras: dict[str, _SimCameraConfig] = {}

        def get_type(t):
            if t == CameraType.fixed:
                return CameraType.fixed
            if t == CameraType.tracking:
                return CameraType.tracking
            if t == CameraType.free:
                return CameraType.free
            return CameraType.default_free

        for name, camera_cfg in cfg.cameras.items():
            cpp_camera_cfg = _SimCameraConfig()
            cpp_camera_cfg.type = get_type(camera_cfg.type)
            cpp_camera_cfg.identifier = camera_cfg.identifier
            cameras[name] = cpp_camera_cfg

        cpp_set_cfg = _SimCameraSetConfig()
        cpp_set_cfg.cameras = cameras
        cpp_set_cfg.resolution_width = cfg.resolution_width
        cpp_set_cfg.resolution_height = cfg.resolution_height
        cpp_set_cfg.frame_rate = cfg.frame_rate
        cpp_set_cfg.max_buffer_frames = cfg.max_buffer_frames

        super().__init__(sim, cpp_set_cfg)
        self._sim: rcsss.sim.Sim

    def get_latest_frames(self) -> FrameSet | None:
        """Should return the latest frame from the camera with the given name."""
        return self._cpp_to_python_frames(super().get_latest_frameset())

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Should return the frame from the camera with the given name and closest to the given timestamp."""
        return self._cpp_to_python_frames(super().get_timestamp_frameset(ts.timestamp()))

    def _cpp_to_python_frames(self, cpp_frameset: _FrameSet | None) -> FrameSet | None:
        if cpp_frameset is None:
            return None
        frames: dict[str, Frame] = {}
        c_frames_iter = cpp_frameset.color_frames.items()
        d_frames_iter = cpp_frameset.depth_frames.items()
        for (color_name, color_frame), (depth_name, depth_frame) in zip(c_frames_iter, d_frames_iter, strict=True):
            assert color_name == depth_name
            color_np_frame = np.copy(color_frame).reshape(self._cfg.resolution_height, self._cfg.resolution_width, 3)[
                # convert from column-major (c++ eigen) to row-major (python numpy)
                ::-1
            ]
            depth_np_frame = np.copy(depth_frame).reshape(self._cfg.resolution_height, self._cfg.resolution_width, 1)[
                # convert from column-major (c++ eigen) to row-major (python numpy)
                ::-1
            ]
            if self._cfg.physical_units:
                # Convert from [0 1] to depth in meters, see links below:
                # http://stackoverflow.com/a/6657284/1461210
                # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
                near = self._sim.model.vis.map.znear
                far = self._sim.model.vis.map.zfar
                depth_np_frame = near / (1 - depth_np_frame * (1 - near / far))
            cameraframe = CameraFrame(
                color=DataFrame(data=color_np_frame, timestamp=cpp_frameset.timestamp),
                depth=DataFrame(data=depth_np_frame, timestamp=cpp_frameset.timestamp),
            )
            frame = Frame(camera=cameraframe, avg_timestamp=cpp_frameset.timestamp)
            frames[color_name] = frame
        return FrameSet(frames=frames, avg_timestamp=cpp_frameset.timestamp)

    def close(self):
        # TODO: this could deregister camera callbacks in simulation
        pass

    @property
    def config(self) -> SimCameraSetConfig:
        return self._cfg

    @property
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras."""
        return list(self._cfg.cameras.keys())

    @property
    def name_to_identifier(self) -> dict[str, str]:
        return self._cfg.name_to_identifier
