"""
Simulation cameras.

Images are rendered in Python, either with MuJoCo's classic OpenGL renderer or with Filament
(``SimConfig(renderer=RendererBackend.FILAMENT)``). They show one of two simulation states:

- ``render_on_demand=True`` (default): the state at the time the frames are requested.
- ``render_on_demand=False``: the state at the last exposure of each camera, like a real camera that
  records at its ``frame_rate``. The simulation captures its state at each camera's frame rate while
  it steps (``Sim.register_state_capture``, cheap: no rendering), and frames are rendered from the
  captured states when they are requested.

In both cases the state is loaded into a separate render-only ``MjData`` and its kinematics are
recomputed, so the image, its time stamp and the camera extrinsics always belong together.
"""

import logging
import math
from collections import deque
from datetime import datetime
from typing import Literal

import cv2
import mujoco
import numpy as np
from rcs._core import common
from rcs._core.sim import RendererBackend, SimCameraConfig
from rcs.camera.interface import BaseCameraSet, CameraFrame, DataFrame, Frame, FrameSet
from rcs.sim import filament, render_context_bootstrap

from rcs import sim

logger = logging.getLogger(__name__)

# state that is captured and restored for rendering
STATE_SPEC = mujoco.mjtState.mjSTATE_INTEGRATION


def _camera_id(model: mujoco.MjModel, cfg: SimCameraConfig) -> int:
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cfg.identifier)
    if cam_id < 0:
        msg = f"camera {cfg.identifier!r} does not exist in the model"
        raise ValueError(msg)
    return cam_id


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


class _ClassicRenderer:
    """MuJoCo's classic OpenGL renderer, offscreen, one context per resolution.

    Unlike ``mujoco.Renderer`` this is not limited by the model's ``offwidth``/``offheight``.
    """

    def __init__(self, model: mujoco.MjModel, physical_units: bool):
        render_context_bootstrap.require("simulation camera rendering")
        render_context_bootstrap.make_current()
        self._model = model
        self._physical_units = physical_units
        self._scene = mujoco.MjvScene(model, maxgeom=2000)
        self._opt = mujoco.MjvOption()
        self._contexts: dict[tuple[int, int], tuple[mujoco.MjrContext, mujoco.MjrRect, np.ndarray, np.ndarray]] = {}
        extent = model.stat.extent
        self._near = model.vis.map.znear * extent
        self._far = model.vis.map.zfar * extent

    def _context(self, width: int, height: int):
        key = (width, height)
        if key not in self._contexts:
            ctx = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
            mujoco.mjr_resizeOffscreen(width, height, ctx)
            rgb = np.empty((height, width, 3), dtype=np.uint8)
            depth = np.empty((height, width), dtype=np.float32)
            self._contexts[key] = (ctx, mujoco.MjrRect(0, 0, width, height), rgb, depth)
        return self._contexts[key]

    def update(self, data: mujoco.MjData):
        pass

    def render(
        self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int, depth: bool
    ) -> tuple[np.ndarray, np.ndarray | None]:
        """Returns (H, W, 3) uint8 RGB and, if requested, (H, W) float32 depth, both top row first.

        Depth is in meters with ``physical_units``, otherwise the raw [0, 1] OpenGL depth buffer.
        """
        render_context_bootstrap.make_current()
        ctx, viewport, rgb_buffer, depth_buffer = self._context(width, height)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
        # The scene is shared by all cameras and mjv_updateScene positions infinite planes (e.g. the
        # floor) from scene.camera *before* it updates the camera, i.e. from the previously rendered
        # camera. Update the camera first so that images do not depend on the render order.
        mujoco.mjv_updateCamera(self._model, data, camera, self._scene)
        mujoco.mjv_updateScene(self._model, data, self._opt, None, camera, mujoco.mjtCatBit.mjCAT_ALL, self._scene)
        mujoco.mjr_render(viewport, self._scene, ctx)
        mujoco.mjr_readPixels(rgb_buffer, depth_buffer if depth else None, viewport, ctx)
        # OpenGL reads bottom-up
        rgb = cv2.flip(rgb_buffer, 0)
        if not depth:
            return rgb, None
        depth_image = cv2.flip(depth_buffer, 0)
        if self._physical_units:
            # Convert from [0 1] to depth in meters, see links below:
            # http://stackoverflow.com/a/6657284/1461210
            # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
            depth_image = self._near / (1 - depth_image * (1 - self._near / self._far))
        return rgb, depth_image

    def close(self):
        if not self._contexts:
            return
        render_context_bootstrap.make_current()
        for ctx, *_ in self._contexts.values():
            ctx.free()
        self._contexts.clear()


class _FilamentRenderer:
    """Adapter of :class:`rcs.sim.filament.FilamentRenderer` (depth is always metric)."""

    def __init__(self, model: mujoco.MjModel):
        self._renderer = filament.FilamentRenderer(model)

    def update(self, data: mujoco.MjData):
        self._renderer.update(data)

    def render(
        self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int, depth: bool
    ) -> tuple[np.ndarray, np.ndarray | None]:
        rgb = self._renderer.render(data, camera, width, height)
        return rgb, self._renderer.render_depth(data, camera, width, height) if depth else None

    def close(self):
        self._renderer.close()


class SimCameraSet:
    """Set of cameras in a MuJoCo simulation, implements BaseCameraSet.

    Rendering uses MuJoCo's classic OpenGL renderer, or Filament if the simulation was configured with
    ``SimConfig(renderer=RendererBackend.FILAMENT)`` (constructing this class then returns a
    :class:`FilamentSimCameraSet`).

    Args:
        simulation: the simulation to render.
        cameras: camera configurations by name. Each renders the camera of the model named by its
            ``identifier``; cameras attached to bodies or with a tracking ``mode`` in MJCF move with the
            simulation.
        physical_units: depth in meters (scaled by ``BaseCameraSet.DEPTH_SCALE``); otherwise the raw
            [0, 1] depth buffer of the classic renderer is scaled.
        render_on_demand: if True, frames show the simulation state at the time they are requested.
            If False, they show the state at the last exposure of each camera according to its
            ``frame_rate``, which the simulation captures while it steps, see module docstring.
        max_buffer_frames: number of rendered framesets (``render_on_demand=True``) or captured states
            per frame rate (``render_on_demand=False``) that are kept for ``get_timestamp_frames``.
        render_depth: render depth images; frames have ``depth=None`` otherwise. Set automatically by
            ``CameraSetWrapper(include_depth=...)``.
    """

    DEPTH_SCALE: int = BaseCameraSet.DEPTH_SCALE

    def __new__(cls, simulation: sim.Sim, *_args, **_kwargs):
        if cls is SimCameraSet and simulation.get_config().renderer == RendererBackend.FILAMENT:
            return super().__new__(FilamentSimCameraSet)
        return super().__new__(cls)

    def __init__(
        self,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = False,
        render_on_demand: bool = True,
        max_buffer_frames: int = 100,
        render_depth: bool = True,
    ):
        if max_buffer_frames <= 0:
            msg = "max_buffer_frames must be positive"
            raise ValueError(msg)
        self._sim = simulation
        self.cameras = cameras
        self.physical_units = physical_units
        self.render_on_demand = render_on_demand
        self.render_depth = render_depth
        self._max_buffer_frames = max_buffer_frames
        self._renderer = self._create_renderer()

        model = self._sim.model
        self._render_data = mujoco.MjData(model)
        self._state = np.empty(mujoco.mj_stateSize(model, STATE_SPEC))
        self._mj_cameras: dict[str, mujoco.MjvCamera] = {}
        for name, cfg in cameras.items():
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam.fixedcamid = _camera_id(model, cfg)
            self._mj_cameras[name] = cam

        # render_on_demand=True: rendered framesets, newest last
        self._buffer: deque[FrameSet] = deque(maxlen=max_buffer_frames)
        self._buffer_states: deque[np.ndarray] = deque(maxlen=max_buffer_frames)
        # render_on_demand=False: one state capture per frame rate and the cameras using it
        self._captures: dict[int, list[str]] = {}
        # rendered frames of captured states by (capture id, time stamp)
        self._capture_frames: dict[tuple[int, float], dict[str, Frame]] = {}
        if not render_on_demand:
            by_rate: dict[int, list[str]] = {}
            for name, cfg in cameras.items():
                by_rate.setdefault(cfg.frame_rate, []).append(name)
            for frame_rate, names in by_rate.items():
                capture_id = self._sim.register_state_capture(frame_rate, max_buffer_frames, int(STATE_SPEC))
                self._captures[capture_id] = names

    def _create_renderer(self) -> _ClassicRenderer | _FilamentRenderer:
        return _ClassicRenderer(self._sim.model, self.physical_units)

    def _load_state(self, state: np.ndarray):
        """Loads a state into the render data and computes what rendering needs from it."""
        model, data = self._sim.model, self._render_data
        mujoco.mj_setState(model, data, state, STATE_SPEC)
        mujoco.mj_kinematics(model, data)
        mujoco.mj_camlight(model, data)
        if model.ntendon:
            mujoco.mj_tendon(model, data)
        if model.nflex:
            mujoco.mj_flex(model, data)
        self._renderer.update(data)

    def _render(self, names: list[str]) -> dict[str, Frame]:
        """Renders the given cameras from the render data."""
        model, data = self._sim.model, self._render_data
        timestamp = data.time
        frames: dict[str, Frame] = {}
        for name in names:
            cfg = self.cameras[name]
            color, depth = self._renderer.render(
                data, self._mj_cameras[name], cfg.resolution_width, cfg.resolution_height, self.render_depth
            )
            intrinsics = _intrinsics(model, cfg)
            extrinsics = _extrinsics(model, data, cfg)
            depth_frame = None
            if depth is not None:
                depth_frame = DataFrame(
                    data=(depth[..., np.newaxis] * BaseCameraSet.DEPTH_SCALE).astype(np.uint16),
                    timestamp=timestamp,
                    intrinsics=intrinsics,
                    extrinsics=extrinsics,
                )
            frames[name] = Frame(
                camera=CameraFrame(
                    color=DataFrame(data=color, timestamp=timestamp, intrinsics=intrinsics, extrinsics=extrinsics),
                    depth=depth_frame,
                ),
                avg_timestamp=timestamp,
            )
        return frames

    def _captured_frames(self, max_timestamp: float) -> FrameSet | None:
        frames: dict[str, Frame] = {}
        for capture_id, names in self._captures.items():
            captured = self._sim.get_captured_state(capture_id, max_timestamp)
            if captured is None:
                return None
            key = (capture_id, captured.timestamp)
            if key not in self._capture_frames:
                self._load_state(captured.state)
                self._capture_frames[key] = self._render(names)
                # keep as many rendered frames as captured states
                while len(self._capture_frames) > self._max_buffer_frames * len(self._captures):
                    del self._capture_frames[next(iter(self._capture_frames))]
            frames.update(self._capture_frames[key])
        timestamps = [frame.avg_timestamp for frame in frames.values() if frame.avg_timestamp is not None]
        return FrameSet(frames=frames, avg_timestamp=float(np.mean(timestamps)) if timestamps else None)

    def get_latest_frames(self) -> FrameSet | None:
        """Frames of the current simulation state, or of the last exposures (``render_on_demand=False``)."""
        if not self.render_on_demand:
            return self._captured_frames(math.inf)
        mujoco.mj_getState(self._sim.model, self._sim.data, self._state, STATE_SPEC)
        if self._buffer_states and np.array_equal(self._buffer_states[-1], self._state):
            return self._buffer[-1]
        self._load_state(self._state)
        frames = self._render(list(self.cameras))
        frameset = FrameSet(frames=frames, avg_timestamp=self._render_data.time)
        self._buffer.append(frameset)
        self._buffer_states.append(self._state.copy())
        return frameset

    def get_timestamp_frames(self, ts: datetime) -> FrameSet | None:
        """Most recent frames with a simulation time <= ts."""
        if not self.render_on_demand:
            return self._captured_frames(ts.timestamp())
        for frameset in reversed(self._buffer):
            if frameset.avg_timestamp is not None and frameset.avg_timestamp <= ts.timestamp():
                return frameset
        return None

    def buffer_size(self) -> int:
        if not self.render_on_demand:
            return min(self._sim.captured_state_count(capture_id) for capture_id in self._captures)
        return len(self._buffer)

    def clear_buffer(self):
        self._buffer.clear()
        self._buffer_states.clear()
        self._capture_frames.clear()
        for capture_id in self._captures:
            self._sim.clear_captured_states(capture_id)

    def calibrate(self) -> bool:
        return True

    def config(self, camera_name: str) -> SimCameraConfig:
        """Should return the configuration of the camera with the given name."""
        return self.cameras[camera_name]

    def close(self):
        # the state captures keep running in the simulation, they are cheap (no rendering)
        self._renderer.close()

    @property
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras."""
        return list(self.cameras.keys())

    @property
    def name_to_identifier(self) -> dict[str, str]:
        return {name: cfg.identifier for name, cfg in self.cameras.items()}


class FilamentSimCameraSet(SimCameraSet):
    """Set of simulation cameras rendered with MuJoCo's Filament renderer (MuJoCo >= 4.0).

    Same as :class:`SimCameraSet`, except that depth images are always metric (scaled by
    ``BaseCameraSet.DEPTH_SCALE``), regardless of ``physical_units``. The depth pass is a second
    Filament render per camera, set ``render_depth=False`` (done automatically by
    ``CameraSetWrapper(include_depth=False)``) to skip it.
    """

    def __init__(
        self,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = True,
        render_on_demand: bool = True,
        max_buffer_frames: int = 100,
        render_depth: bool = True,
    ):
        filament.require("FilamentSimCameraSet")
        if not physical_units:
            logger.warning("FilamentSimCameraSet always returns metric depth; physical_units=False is ignored.")
        super().__init__(simulation, cameras, True, render_on_demand, max_buffer_frames, render_depth)

    def _create_renderer(self) -> _ClassicRenderer | _FilamentRenderer:
        return _FilamentRenderer(self._sim.model)
