import logging
from datetime import datetime
from typing import Literal

import mujoco
import numpy as np

# from rcs._core.common import BaseCameraConfig
from rcs._core import common
from rcs._core.sim import FrameSet as _FrameSet
from rcs._core.sim import SimCameraConfig
from rcs._core.sim import SimCameraSet as _SimCameraSet
from rcs.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, FrameSet

from rcs import sim


class SimCameraSet(_SimCameraSet):
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet
    """

    def __init__(
        self,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = False,
        render_on_demand: bool = True,
    ):
        self._logger = logging.getLogger(__name__)
        self.cameras = cameras
        self.physical_units = physical_units

        super().__init__(simulation, cameras, render_on_demand=render_on_demand)
        self._sim: sim.Sim

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
                color=DataFrame(
                    data=color_np_frame,
                    timestamp=cpp_frameset.timestamp,
                    intrinsics=self._intrinsics(color_name),
                    extrinsics=self._extrinsics(color_name),
                ),
                depth=DataFrame(
                    data=(depth_np_frame * BaseCameraSet.DEPTH_SCALE).astype(np.uint16),
                    timestamp=cpp_frameset.timestamp,
                    intrinsics=self._intrinsics(depth_name),
                    extrinsics=self._extrinsics(depth_name),
                ),
            )
            frame = Frame(camera=cameraframe, avg_timestamp=cpp_frameset.timestamp)
            frames[color_name] = frame
        return FrameSet(frames=frames, avg_timestamp=cpp_frameset.timestamp)

    def _intrinsics(self, camera_name) -> np.ndarray[tuple[Literal[3, 4]], np.dtype[np.float64]]:
        cam_id = mujoco.mj_name2id(self._sim.model, mujoco.mjtObj.mjOBJ_CAMERA, self.cameras[camera_name].identifier)
        fovy = self._sim.model.cam_fovy[cam_id]
        fx = fy = 0.5 * self.cameras[camera_name].resolution_height / np.tan(fovy * np.pi / 360)
        return np.array(
            [
                [fx, 0, (self.cameras[camera_name].resolution_width - 1) / 2, 0],
                [0, fy, (self.cameras[camera_name].resolution_height - 1) / 2, 0],
                [0, 0, 1, 0],
            ]
        )

    def _extrinsics(self, camera_name) -> np.ndarray[tuple[Literal[4, 4]], np.dtype[np.float64]]:
        cam_id = mujoco.mj_name2id(self._sim.model, mujoco.mjtObj.mjOBJ_CAMERA, self.cameras[camera_name].identifier)
        xpos = self._sim.data.cam_xpos[cam_id]
        xmat = self._sim.data.cam_xmat[cam_id].reshape(3, 3)

        cam = common.Pose(rotation=xmat, translation=xpos)
        # put z axis infront
        rotation_p = common.Pose(rpy_vector=[np.pi, 0, 0], translation=[0, 0, 0])
        cam = cam * rotation_p

        return cam.inverse().pose_matrix()

    def calibrate(self) -> bool:
        return True

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
