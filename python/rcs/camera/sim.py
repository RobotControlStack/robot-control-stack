import logging
from datetime import datetime

import numpy as np
import rcs

# from rcs._core.common import BaseCameraConfig
from rcs._core.sim import FrameSet as _FrameSet
from rcs._core.sim import SimCameraConfig
from rcs._core.sim import SimCameraSet as _SimCameraSet
from rcs.camera.interface import CameraFrame, DataFrame, Frame, FrameSet


class SimCameraSet(_SimCameraSet):
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet
    """

    def __init__(
        self,
        sim: rcs.sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = False,
        render_on_demand: bool = True,
    ):
        self._logger = logging.getLogger(__name__)
        self.cameras = cameras
        self.physical_units = physical_units

        super().__init__(sim, cameras, render_on_demand=render_on_demand)
        self._sim: rcs.sim.Sim

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
            color_np_frame = np.copy(color_frame).reshape(
                self.cameras[color_name].resolution_height, self.cameras[color_name].resolution_width, 3
            )[
                # convert from column-major (c++ eigen) to row-major (python numpy)
                ::-1
            ]
            depth_np_frame = np.copy(depth_frame).reshape(
                self.cameras[depth_name].resolution_height, self.cameras[depth_name].resolution_width, 1
            )[
                # convert from column-major (c++ eigen) to row-major (python numpy)
                ::-1
            ]
            if self.physical_units:
                # Convert from [0 1] to depth in meters, see links below:
                # http://stackoverflow.com/a/6657284/1461210
                # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
                # https://github.com/htung0101/table_dome/blob/master/table_dome_calib/utils.py#L160
                extent = self._sim.model.stat.extent
                near = self._sim.model.vis.map.znear * extent
                far = self._sim.model.vis.map.zfar * extent
                depth_np_frame = near / (1 - depth_np_frame * (1 - near / far))
            cameraframe = CameraFrame(
                color=DataFrame(data=color_np_frame, timestamp=cpp_frameset.timestamp),
                depth=DataFrame(data=depth_np_frame, timestamp=cpp_frameset.timestamp),
            )
            frame = Frame(camera=cameraframe, avg_timestamp=cpp_frameset.timestamp)
            frames[color_name] = frame
        return FrameSet(frames=frames, avg_timestamp=cpp_frameset.timestamp)

    def config(self, camera_name: str) -> SimCameraConfig:
        """Should return the configuration of the camera with the given name."""
        return self.cameras[camera_name]

    def close(self):
        # TODO: this could deregister camera callbacks in simulation
        pass

    @property
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras."""
        return list(self.cameras.keys())

    @property
    def name_to_identifier(self) -> dict[str, str]:
        return {name: cfg.identifier for name, cfg in self.cameras.items()}
