"""
Load the EGL library, create a persistent GLContext, and register it with the C++ backend.

Globals prevent the library and context from being garbage-collected.
Call `bootstrap()` to complete initialization.
"""

import ctypes
import ctypes.util
import os

_egl_available = False
_addr_make_current = None
_egl_display = None
_egl_context = None

name = ctypes.util.find_library("EGL")
if name is not None:
    try:
        import mujoco.egl
        from mujoco.egl import GLContext

        _egl = ctypes.CDLL(name, mode=os.RTLD_LOCAL | os.RTLD_NOW)
        _addr_make_current = ctypes.cast(_egl.eglMakeCurrent, ctypes.c_void_p).value
        _ctx = GLContext(max_width=3840, max_height=2160)
        _egl_display = int(mujoco.egl.EGL_DISPLAY.address)
        _egl_context = int(_ctx._context.address)
        _egl_available = True
    except Exception:
        pass


def bootstrap():
    if not _egl_available:
        return
    import rcs._core as _cxx

    assert _addr_make_current is not None
    assert _egl_display is not None
    assert _egl_context is not None
    _cxx.common._bootstrap_egl(_addr_make_current, _egl_display, _egl_context)
