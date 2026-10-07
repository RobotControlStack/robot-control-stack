import logging
import warnings
from datetime import datetime
from typing import Literal

import mujoco
import numpy as np
from rcs._core import common
from rcs._core.sim import CameraType, RendererBackend, SimCameraConfig
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


class _ClassicRenderer:
    """Offscreen RGB and depth rendering with MuJoCo's classic OpenGL renderer.

    Uses one ``MjrContext`` per resolution, resized with ``mjr_resizeOffscreen`` so the images are
    not limited by the model's ``offwidth``/``offheight``.
    """

    def __init__(self, model: mujoco.MjModel):
        render_context_bootstrap.require("simulation camera rendering")
        render_context_bootstrap.make_current()
        self._model = model
        self._scene = mujoco.MjvScene(model, maxgeom=2000)
        self._opt = mujoco.MjvOption()
        self._ctxs: dict[tuple[int, int], mujoco.MjrContext] = {}

    def _context(self, width: int, height: int) -> mujoco.MjrContext:
        key = (width, height)
        if key not in self._ctxs:
            ctx = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)
            mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
            mujoco.mjr_resizeOffscreen(width, height, ctx)
            self._ctxs[key] = ctx
        return self._ctxs[key]

    def render(
        self,
        data: mujoco.MjData,
        camera: mujoco.MjvCamera,
        width: int,
        height: int,
        color: bool = True,
        depth: bool = True,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Returns ``(rgb, depth)`` with top row first; rgb is (H, W, 3) uint8, depth is the raw
        OpenGL depth buffer in [0, 1] as (H, W) float32. Each is None if not requested."""
        render_context_bootstrap.make_current()
        ctx = self._context(width, height)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(self._model, data, self._opt, None, camera, mujoco.mjtCatBit.mjCAT_ALL, self._scene)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, ctx)
        mujoco.mjr_render(viewport, self._scene, ctx)
        rgb = np.empty((height, width, 3), dtype=np.uint8) if color else None
        depth_buffer = np.empty((height, width), dtype=np.float32) if depth else None
        mujoco.mjr_readPixels(rgb, depth_buffer, viewport, ctx)
        # OpenGL reads bottom-up
        return (
            rgb[::-1].copy() if rgb is not None else None,
            depth_buffer[::-1].copy() if depth_buffer is not None else None,
        )

    def close(self):
        for ctx in self._ctxs.values():
            ctx.free()
        self._ctxs.clear()


class SimCameraSet:
    """Represents a set of cameras in a mujoco simulation.
    Implements BaseCameraSet

    Color images are rendered with MuJoCo's classic OpenGL renderer or, if the simulation is configured
    with ``SimConfig(renderer=RendererBackend.FILAMENT)``, with Filament (MuJoCo >= 3.15). Filament
    does not expose a metric depth buffer through MuJoCo's Python API, so depth is always rendered
    with the classic renderer. The depth pass is skipped with ``render_depth=False`` (set automatically
    by ``CameraSetWrapper(include_depth=False)``); frames then have ``depth=None``.

    Timing: by default the simulation records its state at each camera's frame rate while stepping
    and the frames returned by ``get_latest_frames`` are rendered from the latest such snapshot, i.e.
    like a real camera they lag behind the current simulation state by up to one camera period, but
    only frames that are actually requested get rendered. With ``render_current=True`` (or a camera
    frame rate of 0) the current simulation state is rendered instead.
    """

    DEPTH_SCALE: int = BaseCameraSet.DEPTH_SCALE

    def __init__(
        self,
        simulation: sim.Sim,
        cameras: dict[str, SimCameraConfig],
        physical_units: bool = False,
        render_current: bool = False,
        max_buffer_frames: int = 100,
        render_depth: bool = True,
        render_on_demand: bool | None = None,
    ):
        if render_on_demand is not None:
            warnings.warn(
                "render_on_demand is deprecated, use render_current instead", DeprecationWarning, stacklevel=2
            )
            render_current = render_on_demand
        if max_buffer_frames <= 0:
            msg = "max_buffer_frames must be positive"
            raise ValueError(msg)
        self._sim = simulation
        self.cameras = cameras
        self.physical_units = physical_units
        self.render_current = render_current
        self.render_depth = render_depth
        self.renderer: RendererBackend = simulation.get_config().renderer
        self._buffer: list[FrameSet] = []
        self._max_buffer_frames = max_buffer_frames
        # latest rendered frame per camera, reused as long as its source state is unchanged
        self._latest: dict[str, Frame] = {}
        # scratch data to restore state snapshots into for rendering
        self._scratch: mujoco.MjData | None = None

        model = self._sim.model
        self._filament: filament.FilamentRenderer | None = None
        # also renders depth for the Filament backend, created lazily as it needs a GL context
        self._classic: _ClassicRenderer | None = None
        if self.renderer == RendererBackend.FILAMENT:
            self._filament = filament.FilamentRenderer(model)
        else:
            self._classic = _ClassicRenderer(model)

        self._mj_cameras: dict[str, mujoco.MjvCamera] = {}
        for name, cfg in cameras.items():
            cam = mujoco.MjvCamera()
            if cfg.type == CameraType.default_free:
                mujoco.mjv_defaultFreeCamera(model, cam)
            else:
                cam.type = int(cfg.type)
                cam.fixedcamid = _camera_id(model, cfg)
            self._mj_cameras[name] = cam
            if not render_current and cfg.frame_rate > 0:
                self._sim.register_state_snapshots(cfg.frame_rate)

    def _classic_renderer(self) -> _ClassicRenderer:
        if self._classic is None:
            self._classic = _ClassicRenderer(self._sim.model)
        return self._classic

    def _depth_to_output(self, depth: np.ndarray) -> np.ndarray:
        if self.physical_units:
            # Convert from [0 1] to depth in meters, see links below:
            # http://stackoverflow.com/a/6657284/1461210
            # https://www.khronos.org/opengl/wiki/Depth_Buffer_Precision
            # https://github.com/htung0101/table_dome/blob/master/table_dome_calib/utils.py#L160
            extent = self._sim.model.stat.extent
            near = self._sim.model.vis.map.znear * extent
            far = self._sim.model.vis.map.zfar * extent
            depth = near / (1 - depth * (1 - near / far))
        return (depth[..., np.newaxis] * self.DEPTH_SCALE).astype(np.uint16)

    def _update_kinematics(self, data: mujoco.MjData):
        """Computes the positions rendering needs from qpos (no collision detection or dynamics).

        Also used for the live data: after a step, mj_step2 has integrated qpos without recomputing
        positions, so without this the image would lag one physics step behind the state.
        """
        model = self._sim.model
        mujoco.mj_kinematics(model, data)
        mujoco.mj_camlight(model, data)
        if model.ntendon > 0:
            mujoco.mj_tendon(model, data)
        if model.nflex > 0:
            mujoco.mj_flex(model, data)

    def _restore_snapshot(self, state: np.ndarray) -> mujoco.MjData:
        """Restores a state snapshot into the scratch data."""
        model = self._sim.model
        if self._scratch is None:
            self._scratch = mujoco.MjData(model)
        mujoco.mj_setState(
            model, self._scratch, np.asarray(state, dtype=np.float64).ravel(), mujoco.mjtState.mjSTATE_INTEGRATION
        )
        return self._scratch

    def _render_frame(self, name: str, data: mujoco.MjData, timestamp: float) -> Frame:
        model = self._sim.model
        cfg = self.cameras[name]
        cam = self._mj_cameras[name]
        width, height = cfg.resolution_width, cfg.resolution_height
        color: np.ndarray | None
        depth: np.ndarray | None = None
        if self._filament is not None:
            color = self._filament.render(data, cam, width, height)
            if self.render_depth:
                _, depth = self._classic_renderer().render(data, cam, width, height, color=False, depth=True)
        else:
            color, depth = self._classic_renderer().render(data, cam, width, height, depth=self.render_depth)
        assert color is not None
        intrinsics = _intrinsics(model, cfg)
        extrinsics = _extrinsics(model, data, cfg)
        depth_frame = None
        if depth is not None:
            depth_frame = DataFrame(
                data=self._depth_to_output(depth), timestamp=timestamp, intrinsics=intrinsics, extrinsics=extrinsics
            )
        return Frame(
            camera=CameraFrame(
                color=DataFrame(data=color, timestamp=timestamp, intrinsics=intrinsics, extrinsics=extrinsics),
                depth=depth_frame,
            ),
            avg_timestamp=timestamp,
        )

    def _update_frames(self) -> bool:
        """Renders all cameras whose source state changed, returns whether anything was rendered."""
        live = self._sim.data
        # cameras grouped by the state they render from: the latest snapshot of their frame rate,
        # or the current state
        groups: dict[int, list[str]] = {}
        for name, cfg in self.cameras.items():
            rate = cfg.frame_rate if not self.render_current else 0
            groups.setdefault(rate, []).append(name)

        rendered = False
        for rate, names in groups.items():
            snapshot: tuple[float, np.ndarray] | None = self._sim.get_latest_state_snapshot(rate) if rate > 0 else None
            # before the first snapshot (e.g. right after a reset) the current state is rendered
            timestamp = snapshot[0] if snapshot is not None else live.time
            stale = [n for n in names if n not in self._latest or self._latest[n].avg_timestamp != timestamp]
            if not stale:
                continue
            data = self._restore_snapshot(snapshot[1]) if snapshot is not None else live
            self._update_kinematics(data)
            if self._filament is not None:
                self._filament.update(data)
            for name in stale:
                self._latest[name] = self._render_frame(name, data, timestamp)
            rendered = True
        return rendered

    def buffer_size(self) -> int:
        return len(self._buffer)

    def clear_buffer(self):
        self._buffer.clear()

    def get_latest_frames(self) -> FrameSet | None:
        """Returns the latest frames of all cameras, rendering them if their source state changed."""
        if not self._update_frames() and self._buffer:
            return self._buffer[-1]
        frames = {name: self._latest[name] for name in self.cameras}
        timestamps = [frame.avg_timestamp for frame in frames.values() if frame.avg_timestamp is not None]
        frameset = FrameSet(frames=frames, avg_timestamp=float(np.mean(timestamps)) if timestamps else None)
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
        """Should return the configuration of the camera with the given name."""
        return self.cameras[camera_name]

    def close(self):
        if self._classic is not None:
            self._classic.close()
            self._classic = None
        if self._filament is not None:
            self._filament.close()
            self._filament = None

    @property
    def camera_names(self) -> list[str]:
        """Should return a list of the activated human readable names of the cameras."""
        return list(self.cameras.keys())

    @property
    def name_to_identifier(self) -> dict[str, str]:
        return {name: cfg.identifier for name, cfg in self.cameras.items()}
