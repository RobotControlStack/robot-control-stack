"""
Offscreen OpenGL context for rendering simulation cameras from Python.

One persistent MuJoCo ``GLContext`` is created lazily and made current on the calling thread
before rendering (see :mod:`rcs.camera.sim`):

- Linux: EGL (headless), via ``mujoco.egl``
- macOS: CGL, via ``mujoco.GLContext``

``MUJOCO_GL`` is honored on Linux if set (e.g. ``osmesa`` for software rendering).
"""

import os
import sys
from dataclasses import dataclass
from typing import Any


@dataclass
class _RenderBackend:
    available: bool = False
    error: str | None = None
    # reference kept so the GL context is not garbage-collected
    gl_context: Any = None


def _init() -> _RenderBackend:
    try:
        if sys.platform != "darwin" and not os.environ.get("MUJOCO_GL"):
            from mujoco.egl import GLContext
        else:
            from mujoco import GLContext

        gl_context = GLContext(max_width=3840, max_height=2160)
    except Exception as exc:
        return _RenderBackend(error=f"Failed to initialize MuJoCo GL context: {exc!r}")
    return _RenderBackend(available=True, gl_context=gl_context)


_state: _RenderBackend | None = None


def _get_state() -> _RenderBackend:
    global _state  # noqa: PLW0603
    if _state is None:
        _state = _init()
    return _state


def is_available() -> bool:
    return _get_state().available


def failure_reason() -> str | None:
    return _get_state().error


def require(feature: str = "offscreen rendering"):
    state = _get_state()
    if state.available:
        return
    reason = state.error or "unknown rendering initialization failure"
    message = (
        f"A GL render context is required for {feature}, but it is not available. {reason} "
        "If you do not need rendering, run RCS without simulation cameras/viewers. "
        "If you do need headless rendering, install the system EGL/OpenGL runtime libraries."
    )
    raise RuntimeError(message)


def make_current():
    """Makes the offscreen GL context current on the calling thread."""
    require()
    _get_state().gl_context.make_current()
