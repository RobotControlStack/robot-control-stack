"""
Bootstrap the offscreen GL render context used by the classic renderer (python/rcs/camera/sim.py).

Two backends are supported, selected by platform:

- Linux: create a persistent MuJoCo EGL ``GLContext`` (headless).
- macOS: there is no libEGL. Create a persistent MuJoCo ``GLContext``.

Rendering code calls ``make_current()`` before using the context. The initialized state is held in a
module-level ``_state`` (references keep the context from being garbage-collected).
"""

import ctypes.util
import sys
from dataclasses import dataclass
from typing import Any


@dataclass
class _RenderBackend:
    backend: str = "none"  # "none" | "egl" | "current_context"
    available: bool = False
    error: str | None = None
    # reference kept so the GL context is not garbage-collected
    gl_context: Any = None


def _init_current_context() -> _RenderBackend:
    """macOS: no libEGL. Use a MuJoCo GLContext made current on this thread."""
    try:
        from mujoco import GLContext

        gl_context = GLContext(max_width=3840, max_height=2160)
    except Exception as exc:
        return _RenderBackend(error=f"Failed to initialize MuJoCo GL context: {exc!r}")
    return _RenderBackend(backend="current_context", available=True, gl_context=gl_context)


def _init_egl() -> _RenderBackend:
    """Linux/other: create a persistent MuJoCo EGL GLContext."""
    if ctypes.util.find_library("EGL") is None:
        return _RenderBackend(error="Could not find libEGL via ctypes.util.find_library('EGL').")
    try:
        from mujoco.egl import GLContext

        ctx = GLContext(max_width=3840, max_height=2160)
        return _RenderBackend(backend="egl", available=True, gl_context=ctx)
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


def make_current():
    """Makes the bootstrapped GL context current on the calling thread (for rendering from Python)."""
    require()
    assert _state.gl_context is not None
    _state.gl_context.make_current()
