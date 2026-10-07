"""
Filament rendering backend (MuJoCo >= 3.15).

MuJoCo ships its Filament-based renderer only inside the Python extension modules
(``mujoco._render_filament`` and ``mujoco.experimental.studio``); ``libmujoco`` does not
export the ``mjrf_*`` C API. Everything Filament related therefore lives on the Python
side of RCS:

- :class:`FilamentRenderer` renders RGB images of fixed/free MuJoCo cameras offscreen.
  Filament does not expose a metric depth buffer through the Python API (its depth draw
  mode is an 8-bit visualization), so depth is still produced by the classic renderer.
- :func:`gui_loop` runs MuJoCo Studio (the Filament viewer) in the RCS GUI subprocess.
  Studio must own the main thread of its process and, unlike ``mujoco.viewer``, must *not*
  run under ``mjpython`` on macOS.

Enable it via ``SimConfig(renderer=RendererBackend.FILAMENT)``.

Environment variables:

- ``RCS_FILAMENT_GRAPHICS_API``: ``opengl`` or ``vulkan`` (default: opengl)
- ``RCS_FILAMENT_SOFTWARE_RENDERING=1``: force software rendering for offscreen cameras
- ``RCS_FILAMENT_GFX``: Studio graphics mode for the GUI, e.g. ``opengl``, ``vulkan``, ``web``
  (``web`` serves the viewer over http instead of opening a native window)
"""

import os
import sys
import threading
from logging import getLogger
from tempfile import NamedTemporaryFile

import mujoco
import numpy as np
from rcs._core.sim import GuiClient as _GuiClient
from rcs.utils import SimpleFrameRate

logger = getLogger(__name__)

# Target frames per second of the GUI loop
FPS = 60


def _import_error() -> str | None:
    # NOTE: do not import mujoco.experimental.studio.window here: it registers Filament resource
    # providers that MuJoCo Studio registers again, which crashes the GUI process (see gui_loop)
    try:
        import mujoco._render_filament
        import mujoco.experimental.studio  # noqa: F401
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
        f"since version 3.15 (installed: {mujoco.__version__}). Import failed with: {error}"
    )
    raise RuntimeError(msg)


class FilamentRenderer:
    """Offscreen RGB renderer for a MuJoCo model based on Filament.

    All methods must be called from the thread that created the renderer: Filament asserts
    thread affinity when its engine is destroyed.
    """

    def __init__(self, model: mujoco.MjModel):
        require("FilamentRenderer")
        import mujoco._render_filament as mjrf  # type: ignore[import-not-found]

        # importing the studio window module registers Filament's built-in assets
        # (materials such as pbr.filamat); without it context creation fails. It must not be
        # imported in a process that runs MuJoCo Studio (see _import_error)
        import mujoco.experimental.studio.window  # noqa: F401

        self._mjrf = mjrf
        self.model = model
        self._ctx = mjrf.Context(self._context_config())
        self._ctx.set_clear_color(np.zeros(3, dtype=np.float32))
        self._objects = mjrf.ModelObjects(self._ctx, model)
        self._scene = self._ctx.create_scene(mjrf.SceneParams())
        self._scene.configure_from_model(model)
        self._lights = mjrf.ModelLights(self._scene, self._objects)
        self._renderables = mjrf.ModelRenderables(self._scene, self._objects)
        # one render target per resolution, created lazily
        self._targets: dict[tuple[int, int], tuple[object, object, np.ndarray]] = {}
        self._closed = False

    def _context_config(self):
        mjrf = self._mjrf
        cfg = mjrf.ContextConfig()
        # OpenGL like MuJoCo's own samples; the platform default resolves to Vulkan on some systems
        # (e.g. macOS) where it is not usable
        cfg.graphics_api = mjrf.GraphicsApi.GRAPHICS_API_OPENGL
        api = os.environ.get("RCS_FILAMENT_GRAPHICS_API", "").lower()
        if api == "vulkan":
            cfg.graphics_api = mjrf.GraphicsApi.GRAPHICS_API_VULKAN
        elif api not in ("", "opengl"):
            logger.warning("Unknown RCS_FILAMENT_GRAPHICS_API=%r, using OpenGL", api)
        cfg.force_software_rendering = os.environ.get("RCS_FILAMENT_SOFTWARE_RENDERING", "") == "1"
        return cfg

    def set_options(self, opt: mujoco.MjvOption):
        self._renderables.set_options(opt)

    def update(self, data: mujoco.MjData):
        """Syncs lights and renderables with the current simulation state."""
        self._lights.update(data)
        self._renderables.update(data)

    def _target(self, width: int, height: int):
        key = (width, height)
        if key not in self._targets:
            mjrf = self._mjrf
            target = self._ctx.create_render_target(
                mjrf.RenderTargetConfig(color_format=mjrf.PixelFormat.PIXEL_FORMAT_RGB8)
            )
            target.resize(width, height)
            buffer = np.empty((height, width, 3), dtype=np.uint8)
            read = mjrf.ReadPixelsRequest()
            read.target = target
            read.set_buffer(buffer)
            self._targets[key] = (target, read, buffer)
        return self._targets[key]

    def render(self, data: mujoco.MjData, camera: mujoco.MjvCamera, width: int, height: int) -> np.ndarray:
        """Renders the given camera and returns an (H, W, 3) uint8 RGB image (top row first).

        Call :meth:`update` first whenever ``data`` changed.
        """
        mjrf = self._mjrf
        target, read, buffer = self._target(width, height)
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
            draw_mode=mjrf.DrawMode.DRAW_MODE_DEFAULT,
        )
        request.scene = self._scene
        request.target = target
        request.viewport.left = 0
        request.viewport.bottom = 0
        request.viewport.width = width
        request.viewport.height = height
        # the bindings currently support only a single read request per render call
        frame = self._ctx.render([request], [read])
        self._ctx.wait_for_frame(frame)
        return buffer.copy()

    def close(self):
        if self._closed:
            return
        self._closed = True
        # destroy in reverse creation order; the context must outlive everything it created
        self._targets.clear()
        del self._renderables, self._lights, self._scene, self._objects
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
    from mujoco.experimental.studio import (
        launch_native,
        launch_thread,
        viewer_app,
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
        # ViewerApp provides the interactive Studio GUI (camera selection, visualization options,
        # dragging bodies, ...); without it only the bare scene is rendered. The camera selection
        # lives in the toolbar, which Studio hides by default ([ and ] cycle cameras regardless).
        app = viewer_app.ViewerApp(viewer_app.ViewerAppConfig(show_toolbar=True))
        launch_native.run_native_viewer(config, viewer_endpoint, plugins=[app])
    finally:
        viewer_closed.set()
        sim_thread.join(timeout=5)
