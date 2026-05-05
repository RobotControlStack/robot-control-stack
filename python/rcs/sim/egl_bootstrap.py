"""
Bootstrap the rendering backend used by the C++ simulation camera path.

Linux uses MuJoCo's EGL backend. macOS uses a persistent MuJoCo GLContext
that is made current on the Python thread before C++ creates rendering
contexts.
"""

import ctypes
import ctypes.util
import os
import sys

_egl_available = False
_addr_make_current = None
_egl_display = None
_egl_context = None
_backend = None
_gl_context = None

if sys.platform == "darwin":
    try:
        import mujoco

        _gl_context = mujoco.GLContext(3840, 2160)
        _backend = "gl_context"
    except Exception:
        pass
else:
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
            _backend = "egl"
        except Exception:
            pass


def bootstrap():
    import rcs._core as _cxx

    if _backend == "gl_context":
        assert _gl_context is not None
        _gl_context.make_current()
        _cxx.common._bootstrap_gl_context()
        return

    if _backend == "egl":
        assert _addr_make_current is not None
        assert _egl_display is not None
        assert _egl_context is not None
        _cxx.common._bootstrap_egl(_addr_make_current, _egl_display, _egl_context)
