import logging
from datetime import datetime
from typing import Literal

import mujoco
import numpy as np

# from rcs._core.common import BaseCameraConfig
from rcs._core import common
from rcs._core.sim import CameraType
from rcs._core.sim import FrameSet as _FrameSet
from rcs._core.sim import RendererBackend, SimCameraConfig
from rcs._core.sim import SimCameraSet as _SimCameraSet
from rcs.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, FrameSet
from rcs.sim import filament, render_context_bootstrap

from rcs import sim

logger = logging.getLogger(__name__)


def _camera_id(model: mujoco.MjModel, cfg: SimCameraConfig) -> int:
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cfg.identifier)


def _intrinsics(
    model: mujoco.MjModel, cfg: SimCameraConfig
) -> np.ndarray[tuple[Literal[3], Literal[4]], np.dtype[np.float64]]:
    fovy = model.cam_fovy[_camera_id(model, cfg)]
    fx = fy = 0.5 * cfg.resolution_height / np.tan(fovy * np.pi / 360)
    return np.array(
        [
            [fx, 0, (cfg.resolution_width - 1) / 2, 0],
            [0, fy, (cfg.resolution_height - 1) / 2, 0],
            [0, 0, 1, 0],
        ]
    )


def _extrinsics(
    model: mujoco.MjModel, data: mujoco.MjData, cfg: SimCameraConfig
) -> np.ndarray[tuple[Literal[4], Literal[4]], np.dtype[np.float64]]:
    cam_id = _camera_id(model, cfg)
    xpos = data.cam_xpos[cam_id]
    xmat = data.cam_xmat[cam_id].reshape(3, 3)

    cam = common.Pose(rotation=xmat, translation=xpos)
    # put z axis infront
    rotation_p = common.Pose(rpy_vector=np.array([np.pi, 0, 0]), translation=np.array([0, 0, 0]))  # type: ignore
    cam = cam * rotation_p

    return cam.inverse().pose_matrix()


class SimCameraSet(_SimCameraSet):
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet

    Rendering happens in C++ with MuJoCo's classic OpenGL renderer. If the simulation was configured with
    ``SimConfig(renderer=RendererBackend.FILAMENT)``, constructing this class returns a
    :class:`FilamentSimCameraSet` instead.
    """

    def __new__(
        cls,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = False,
        render_on_demand: bool = True,
    ):
        if simulation.get_config().renderer == RendererBackend.FILAMENT:
            return FilamentSimCameraSet(simulation, cameras, physical_units, render_on_demand)
        return super().__new__(cls)

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

        render_context_bootstrap.require("simulation camera rendering")
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

    def _intrinsics(self, camera_name) -> np.ndarray[tuple[Literal[3], Literal[4]], np.dtype[np.float64]]:
        return _intrinsics(self._sim.model, self.cameras[camera_name])

    def _extrinsics(self, camera_name) -> np.ndarray[tuple[Literal[4], Literal[4]], np.dtype[np.float64]]:
        return _extrinsics(self._sim.model, self._sim.data, self.cameras[camera_name])

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


class _ClassicDepthRenderer:
    """Renders metric depth with MuJoCo's classic renderer from Python.

    Unlike ``mujoco.Renderer`` this is not limited by the model's ``offwidth``/``offheight``.
    """

    def __init__(self, model: mujoco.MjModel, width: int, height: int):
        render_context_bootstrap.require("simulation depth rendering")
        render_context_bootstrap.make_current()
        self._model = model
        self._scene = mujoco.MjvScene(model, maxgeom=2000)
        self._opt = mujoco.MjvOption()
        self._ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._ctx)
        mujoco.mjr_resizeOffscreen(width, height, self._ctx)
        self._viewport = mujoco.MjrRect(0, 0, width, height)
        self._depth = np.empty((height, width), dtype=np.float32)
        extent = model.stat.extent
        self._near = model.vis.map.znear * extent
        self._far = model.vis.map.zfar * extent

    def render(self, data: mujoco.MjData, camera: mujoco.MjvCamera) -> np.ndarray:
        """Returns metric depth in meters as (H, W) float32, top row first."""
        render_context_bootstrap.make_current()
        mujoco.mjv_updateScene(self._model, data, self._opt, None, camera, mujoco.mjtCatBit.mjCAT_ALL, self._scene)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._ctx)
        mujoco.mjr_render(self._viewport, self._scene, self._ctx)
        mujoco.mjr_readPixels(None, self._depth, self._viewport, self._ctx)
        # OpenGL reads bottom-up; convert the [0, 1] depth buffer to meters (see SimCameraSet)
        depth = self._depth[::-1]
        return self._near / (1 - depth * (1 - self._near / self._far))

    def close(self):
        self._ctx.free()


class FilamentSimCameraSet:
    """Set of simulation cameras rendered with MuJoCo's Filament renderer (MuJoCo >= 3.15).
    Implements BaseCameraSet

    Color images come from Filament. Filament does not expose a metric depth buffer through
    MuJoCo's Python API, so depth images are rendered with the classic renderer; they are always
    metric (scaled by ``BaseCameraSet.DEPTH_SCALE``), regardless of ``physical_units``.

    Rendering happens in Python when frames are requested, i.e. always "on demand"; rendering at a
    fixed camera frame rate while the simulation steps is not supported with this backend.
    """

    def __init__(
        self,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = True,
        render_on_demand: bool = True,
        max_buffer_frames: int = 100,
    ):
        filament.require("FilamentSimCameraSet")
        if not physical_units:
            logger.warning("FilamentSimCameraSet always returns metric depth; physical_units=False is ignored.")
        if not render_on_demand:
            logger.warning("FilamentSimCameraSet always renders on demand; render_on_demand=False is ignored.")
        if max_buffer_frames <= 0:
            msg = "max_buffer_frames must be positive"
            raise ValueError(msg)
        self._sim = simulation
        self.cameras = cameras
        self.physical_units = physical_units
        self.render_on_demand = True
        self._buffer: list[FrameSet] = []
        self._max_buffer_frames = max_buffer_frames

        self._renderer = filament.FilamentRenderer(self._sim.model)
        self._mj_cameras: dict[str, mujoco.MjvCamera] = {}
        for name, cfg in cameras.items():
            cam = mujoco.MjvCamera()
            if cfg.type == CameraType.default_free:
                mujoco.mjv_defaultFreeCamera(self._sim.model, cam)
            else:
                cam.type = int(cfg.type)
                cam.fixedcamid = _camera_id(self._sim.model, cfg)
            self._mj_cameras[name] = cam
        # classic renderers for metric depth, one per resolution
        self._depth_renderers: dict[tuple[int, int], _ClassicDepthRenderer] = {}

    def _depth_renderer(self, cfg: SimCameraConfig) -> _ClassicDepthRenderer:
        key = (cfg.resolution_width, cfg.resolution_height)
        if key not in self._depth_renderers:
            self._depth_renderers[key] = _ClassicDepthRenderer(self._sim.model, *key)
        return self._depth_renderers[key]

    def _render(self) -> FrameSet:
        model, data = self._sim.model, self._sim.data
        timestamp = data.time
        self._renderer.update(data)
        frames: dict[str, Frame] = {}
        for name, cfg in self.cameras.items():
            color = self._renderer.render(data, self._mj_cameras[name], cfg.resolution_width, cfg.resolution_height)
            depth = self._depth_renderer(cfg).render(data, self._mj_cameras[name])[..., np.newaxis]
            intrinsics = _intrinsics(model, cfg)
            extrinsics = _extrinsics(model, data, cfg)
            frames[name] = Frame(
                camera=CameraFrame(
                    color=DataFrame(data=color, timestamp=timestamp, intrinsics=intrinsics, extrinsics=extrinsics),
                    depth=DataFrame(
                        data=(depth * BaseCameraSet.DEPTH_SCALE).astype(np.uint16),
                        timestamp=timestamp,
                        intrinsics=intrinsics,
                        extrinsics=extrinsics,
                    ),
                ),
                avg_timestamp=timestamp,
            )
        return FrameSet(frames=frames, avg_timestamp=timestamp)

    def buffer_size(self) -> int:
        return len(self._buffer)

    def clear_buffer(self):
        self._buffer.clear()

    def get_latest_frames(self) -> FrameSet | None:
        """Renders all cameras for the current simulation state and returns the frames."""
        if self._buffer and self._buffer[-1].avg_timestamp == self._sim.data.time:
            return self._buffer[-1]
        frameset = self._render()
        self._buffer.append(frameset)
        del self._buffer[: -self._max_buffer_frames]
        return frameset

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Returns the most recent buffered frames with a simulation time <= ts."""
        for frameset in reversed(self._buffer):
            if frameset.avg_timestamp is not None and frameset.avg_timestamp <= ts.timestamp():
                return frameset
        return None

    def calibrate(self) -> bool:
        return True

    def config(self, camera_name: str) -> SimCameraConfig:
        return self.cameras[camera_name]

    def close(self):
        for renderer in self._depth_renderers.values():
            renderer.close()
        self._depth_renderers.clear()
        self._renderer.close()

    @property
    def camera_names(self) -> list[str]:
        return list(self.cameras.keys())

    @property
    def name_to_identifier(self) -> dict[str, str]:
        return {name: cfg.identifier for name, cfg in self.cameras.items()}
