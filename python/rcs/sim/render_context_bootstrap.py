"""
Bootstrap an offscreen GL render context and register it with the C++ backend.

Two backends are supported, selected by platform:

- Linux: load libEGL via ctypes, create a persistent MuJoCo EGL ``GLContext``,
  and hand its ``eglMakeCurrent`` / display / context to C++ (the ``egl``
  backend).
- macOS: there is no libEGL. Create a persistent MuJoCo ``GLContext`` and make
  it current on this thread; C++ then treats ``ensure_current()`` as a no-op
  (the ``current_context`` backend).

The initialized state is held in a module-level ``_state`` (references keep the
library and context from being garbage-collected). Call ``bootstrap()`` to
complete initialization.
"""

import ctypes
import ctypes.util
import os
import sys
from dataclasses import dataclass
from typing import Any


@dataclass
class _RenderBackend:
    backend: str = "none"  # "none" | "egl" | "current_context"
    available: bool = False
    error: str | None = None
    # references kept so the GL context / EGL library are not garbage-collected
    gl_context: Any = None
    egl_lib: Any = None
    addr_make_current: int | None = None
    egl_display: int | None = None
    egl_context: int | None = None


def _init_current_context() -> _RenderBackend:
    """macOS: no libEGL. Use a MuJoCo GLContext made current on this thread."""
    try:
        from mujoco import GLContext

        gl_context = GLContext(max_width=3840, max_height=2160)
    except Exception as exc:
        return _RenderBackend(error=f"Failed to initialize MuJoCo GL context: {exc!r}")
    return _RenderBackend(backend="current_context", available=True, gl_context=gl_context)


def _init_egl() -> _RenderBackend:
    """Linux/other: load libEGL and create a persistent MuJoCo EGL GLContext."""
    name = ctypes.util.find_library("EGL")
    if name is None:
        return _RenderBackend(error="Could not find libEGL via ctypes.util.find_library('EGL').")
    try:
        import mujoco.egl
        from mujoco.egl import GLContext

        egl_lib = ctypes.CDLL(name, mode=os.RTLD_LOCAL | os.RTLD_NOW)
        addr_make_current = ctypes.cast(egl_lib.eglMakeCurrent, ctypes.c_void_p).value
        ctx = GLContext(max_width=3840, max_height=2160)
        return _RenderBackend(
            backend="egl",
            available=True,
            gl_context=ctx,
            egl_lib=egl_lib,
            addr_make_current=addr_make_current,
            egl_display=int(mujoco.egl.EGL_DISPLAY.address),
            egl_context=int(ctx._context.address),
        )
    except Exception as exc:
        return _RenderBackend(error=f"Failed to initialize MuJoCo EGL context: {exc!r}")


_state = _init_current_context() if sys.platform == "darwin" else _init_egl()


def is_available() -> bool:
    return _state.available


def failure_reason() -> str | None:
    return _state.error


def require(feature: str = "offscreen rendering"):
    if _state.available:
        return
    reason = _state.error or "unknown rendering initialization failure"
    message = (
        f"A GL render context is required for {feature}, but it is not available. {reason} "
        "If you do not need rendering, run RCS without simulation cameras/viewers. "
        "If you do need headless rendering, install the system EGL/OpenGL runtime libraries."
    )
    raise RuntimeError(message)


def bootstrap():
    if not _state.available:
        return
    import rcs._core as _cxx

    if _state.backend == "current_context":
        assert _state.gl_context is not None
        _state.gl_context.make_current()
        _cxx.common._bootstrap_gl_context()
    elif _state.backend == "egl":
        assert _state.addr_make_current is not None
        assert _state.egl_display is not None
        assert _state.egl_context is not None
        _cxx.common._bootstrap_egl_context(_state.addr_make_current, _state.egl_display, _state.egl_context)
