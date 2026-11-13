"""
Filament rendering backend (MuJoCo >= 4.0).

MuJoCo ships its Filament-based renderer only inside the ``mujoco-studio`` Python package
(``mujoco.mjrf`` and ``mujoco.studio``); ``libmujoco`` does not export the ``mjrf_*`` C API.
Everything Filament related therefore lives on the Python side of RCS:

- :class:`FilamentRenderer` renders RGB images and metric depth of fixed/free MuJoCo cameras
  offscreen. Depth comes from Filament's depth draw mode read back into a float target.
- :func:`gui_loop` runs MuJoCo Studio (the Filament viewer) in the RCS GUI subprocess.
  Studio must own the main thread of its process and, unlike ``mujoco.viewer``, must *not*
  run under ``mjpython`` on macOS.

Enable it via ``SimConfig(renderer=RendererBackend.FILAMENT)``.

Environment variables:

- ``RCS_FILAMENT_GRAPHICS_API``: ``opengl`` or ``vulkan`` (default: opengl, vulkan on Linux without an
  X display: Filament's OpenGL backend needs X and hangs without it)
- ``RCS_FILAMENT_SOFTWARE_RENDERING=1``: force software rendering for offscreen cameras
- ``RCS_FILAMENT_GFX``: Studio graphics mode for the GUI, e.g. ``opengl``, ``vulkan``, ``web``
  (``web`` serves the viewer over http instead of opening a native window)
"""

import atexit
import ctypes
import os
import sys
import threading
from logging import getLogger
from tempfile import NamedTemporaryFile

import cv2
import mujoco
import numpy as np
from rcs._core.sim import GuiClient as _GuiClient
from rcs.utils import SimpleFrameRate

logger = getLogger(__name__)

# Target frames per second of the GUI loop
FPS = 60


def _import_error() -> str | None:
    try:
        import mujoco.studio  # noqa: F401
        from mujoco import mjrf  # noqa: F401
    except ImportError as exc:
        return repr(exc)
    return None


def is_available() -> bool:
    return _import_error() is None


def require(feature: str = "Filament rendering"):
    error = _import_error()
    if error is None:
        return
    msg = (
        f"{feature} requires the Filament renderer, which is shipped with the MuJoCo Python package "
        f"(mujoco-studio) since version 4.0 (installed: {mujoco.__version__}). Import failed with: {error}"
    )
    raise RuntimeError(msg)


def _graphics_api(mjrf):
    """Graphics API for offscreen Filament contexts, see ``RCS_FILAMENT_GRAPHICS_API``."""
    api = os.environ.get("RCS_FILAMENT_GRAPHICS_API", "").lower()
    if api not in ("", "opengl", "vulkan"):
        logger.warning("Unknown RCS_FILAMENT_GRAPHICS_API=%r, using the default", api)
        api = ""
    # Filament's OpenGL backend on Linux uses GLX: without an X display it prints "Failed to open X
    # display. (exiting)" and then hangs instead of exiting, so headless Linux has to use Vulkan
    headless_linux = sys.platform.startswith("linux") and not os.environ.get("DISPLAY")
    if api == "opengl" and headless_linux:
        msg = (
            "RCS_FILAMENT_GRAPHICS_API=opengl requires an X display on Linux, but DISPLAY is not set. "
            "Use RCS_FILAMENT_GRAPHICS_API=vulkan (the default without a display) or run under an X server."
        )
        raise RuntimeError(msg)
    if api == "vulkan" or (api == "" and headless_linux):
        return mjrf.GraphicsApi.GRAPHICS_API_VULKAN
    # OpenGL like MuJoCo's own samples; the platform default resolves to Vulkan on some systems
    # (e.g. macOS) where it is not usable
    return mjrf.GraphicsApi.GRAPHICS_API_OPENGL


# Filament contexts that are not in use, per (thread, graphics API, software rendering).
# Contexts are reused instead of destroyed: destroying a Vulkan context on NVIDIA drivers calls
# eglTerminate() on the process' default EGL display (from libGLX_nvidia on Filament's engine thread),
# which kills the EGL context of the classic renderer (eglMakeCurrent then fails with
# EGL_NOT_INITIALIZED). They are keyed by thread as Filament requires all calls of a context to come
# from the thread that created it.
_free_contexts: dict[tuple[int, object, bool], list] = {}
_free_contexts_lock = threading.Lock()


def _acquire_context(mjrf):
    software = os.environ.get("RCS_FILAMENT_SOFTWARE_RENDERING", "") == "1"
    key = (threading.get_ident(), _graphics_api(mjrf), software)
    with _free_contexts_lock:
        free = _free_contexts.setdefault(key, [])
        ctx = free.pop() if free else None
    if ctx is None:
        cfg = mjrf.ContextConfig()
        cfg.graphics_api = key[1]
        cfg.software_rendering = software
        ctx = mjrf.Context(cfg)
    ctx.set_clear_color(np.zeros(3, dtype=np.float32))
    return key, ctx


def _release_context(key, ctx):
    with _free_contexts_lock:
        _free_contexts.setdefault(key, []).append(ctx)


@atexit.register
def _leak_free_contexts():
    # Pooled contexts would otherwise be destroyed during interpreter shutdown on the main thread,
    # which aborts the process for contexts created by other threads ("Engine::shutdown() called from
    # the wrong thread!"). Keep them alive instead; the OS frees them when the process exits.
    with _free_contexts_lock:
        for contexts in _free_contexts.values():
            for ctx in contexts:
                ctypes.pythonapi.Py_IncRef(ctypes.py_object(ctx))


class FilamentRenderer:
    """Offscreen RGB renderer for a MuJoCo model based on Filament.

    All methods must be called from the thread that created the renderer: Filament asserts
    thread affinity when its engine is destroyed. The Filament context is borrowed from a per-thread
    pool and returned on :meth:`close` (a context can only serve one model at a time).
    """

    def __init__(self, model: mujoco.MjModel):
        require("FilamentRenderer")
        from mujoco import mjrf

        self._mjrf = mjrf
        self.model = model
        self._ctx_key, self._ctx = _acquire_context(mjrf)
        self._objects = mjrf.ModelResources(self._ctx, model)
        self._scene = self._ctx.create_scene(mjrf.SceneParams())
        self._scene.configure_from_model(model)
        self._lights = mjrf.ModelLights(self._scene, self._objects)
        self._renderables = mjrf.ModelRenderables(self._scene, self._objects)
        # one render target per (resolution, kind), created lazily
        self._targets: dict[tuple[int, int, str], tuple[object, object, np.ndarray]] = {}
        extent = model.stat.extent
        self._near = model.vis.map.znear * extent
        self._far = model.vis.map.zfar * extent
        self._closed = False

    def set_options(self, opt: mujoco.MjvOption):
        self._renderables.set_options(opt)

    def update(self, data: mujoco.MjData):
        """Syncs lights and renderables with the current simulation state."""
        self._lights.update(data)
        self._renderables.update(data)

    def _target(self, width: int, height: int, kind: str = "rgb"):
        key = (width, height, kind)
        if key not in self._targets:
            mjrf = self._mjrf
            if kind == "rgb":
                # RGBA8 instead of RGB8: GPUs generally do not support 3 channel 8 bit render targets
                # (e.g. Vulkan's R8G8B8_UNORM has no image features on NVIDIA), so Filament emulates
                # RGB8 with a slow readback path (about 2x slower on Vulkan); alpha is dropped in render()
                fmt, buffer = mjrf.PixelFormat.PIXEL_FORMAT_RGBA8, np.empty((height, width, 4), dtype=np.uint8)
            else:
                fmt, buffer = mjrf.PixelFormat.PIXEL_FORMAT_R32F, np.empty((height, width), dtype=np.float32)
            target = self._ctx.create_render_target(mjrf.RenderTargetConfig(color_format=fmt))
            target.resize(width, height)
            read = mjrf.ReadRequest(target=target)
            read.set_buffer(buffer)
            self._targets[key] = (target, read, buffer)
        return self._targets[key]

    def render(self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int) -> np.ndarray:
        """Renders the given camera and returns an (H, W, 3) uint8 RGB image (top row first).

        Call :meth:`update` first whenever ``data`` changed.
        """
        # cv2 drops the alpha channel ~100x faster than a numpy slice copy
        return cv2.cvtColor(self._render(data, camera, width, height, "rgb"), cv2.COLOR_RGBA2RGB)

    def render_depth(self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int) -> np.ndarray:
        """Renders the given camera and returns metric depth in meters as (H, W) float32 (top row first).

        Call :meth:`update` first whenever ``data`` changed.
        """
        depth = self._render(data, camera, width, height, "depth")
        # Filament's depth draw mode writes reversed-z (1 at znear, 0 at zfar) without post processing
        # (MuJoCo >= 4.0); convert like the classic renderer's [0, 1] depth buffer
        return self._near / (1 - (1 - depth) * (1 - self._near / self._far))

    def _render(self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int, kind: str):
        mjrf = self._mjrf
        target, read, buffer = self._target(width, height, kind)
        glcam = mujoco.mjv_camera2GLCamera(self.model, data, camera)
        request = mjrf.RenderRequest(
            camera=mjrf.Camera(
                pos=glcam.pos,
                forward=glcam.forward,
                up=glcam.up,
                frustum_bottom=glcam.frustum_bottom,
                frustum_top=glcam.frustum_top,
                frustum_near=glcam.frustum_near,
                frustum_far=glcam.frustum_far,
            ),
            draw_mode=mjrf.DrawMode.DRAW_MODE_DEFAULT if kind == "rgb" else mjrf.DrawMode.DRAW_MODE_DEPTH,
        )
        request.scene = self._scene
        request.target = target
        request.viewport.left = 0
        request.viewport.bottom = 0
        request.viewport.width = width
        request.viewport.height = height
        # the bindings still support only a single read request per render call (MuJoCo 4.0.1)
        frame = self._ctx.render([request], [read])
        self._ctx.wait_for_frame(frame)
        return buffer

    def close(self):
        if self._closed:
            return
        self._closed = True
        # destroy in reverse creation order; the context must outlive everything it created
        self._targets.clear()
        del self._renderables, self._lights, self._scene, self._objects
        # the context is not destroyed but reused, see _free_contexts
        _release_context(self._ctx_key, self._ctx)
        del self._ctx


def gui_executable() -> str:
    """Python executable for the Filament GUI subprocess (never mjpython, see module docstring)."""
    return sys.executable


def gui_loop(gui_uuid: str, close_event):
    """Runs MuJoCo Studio for the simulation identified by ``gui_uuid``.

    Mirrors :func:`rcs.sim.sim.gui_loop`: the GUI process holds its own copy of the model and data
    and synchronizes the state through the GuiClient. The viewer runs on the main thread of this
    process, stepping/syncing happens on a background thread.
    """
    require("The Filament GUI")
    from mujoco.studio import app as studio_app
    from mujoco.studio import (
        launch_native,
        launch_thread,
        viewer_handle,
        viewer_protocol,
    )

    gui_client = _GuiClient(gui_uuid)
    model_bytes = gui_client.get_model_bytes()
    with NamedTemporaryFile(mode="wb") as f:
        f.write(model_bytes)
        f.flush()
        model = mujoco.MjModel.from_binary_path(f.name)
    data = mujoco.MjData(model)
    gui_client.set_model_and_data(model._address, data._address)
    mujoco.mj_step(model, data)

    viewer_endpoint, sim_endpoint = launch_thread.make_thread_endpoints()
    handle = viewer_handle.ViewerHandle(sim_endpoint)
    viewer_closed = threading.Event()

    def sim_loop():
        frame_rate = SimpleFrameRate(FPS, "gui_loop")
        try:
            while not close_event.is_set() and not viewer_closed.is_set() and handle.is_running():
                mujoco.mj_step(model, data)
                # the viewer may hand back a new model (e.g. drag and drop); we keep ours as the
                # GuiClient owns the pointers to model and data
                handle.sync(model, data)
                gui_client.sync()
                frame_rate()
        finally:
            handle.close()

    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()
    try:
        config = viewer_protocol.ViewerConfig(title="RCS", gfx=os.environ.get("RCS_FILAMENT_GFX", ""))
        # App provides the interactive Studio GUI (camera selection, visualization options,
        # dragging bodies, ...); without it only the bare scene is rendered. The camera selection
        # lives in the toolbar, which Studio hides by default ([ and ] cycle cameras regardless).
        # Studio's SIM_PLUGINS (step control, history) are not used: sim_loop steps and paces itself.
        app = studio_app.App(studio_app.Config(show_toolbar=True))
        launch_native.run_native_viewer(config, viewer_endpoint, plugins=[app])
    finally:
        viewer_closed.set()
        sim_thread.join(timeout=5)
