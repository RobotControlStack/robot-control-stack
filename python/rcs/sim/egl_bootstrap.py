"""
Load the EGL library, create a persistent GLContext, and register it with the C++ backend.

Globals prevent the library and context from being garbage-collected.  
Call `bootstrap()` to complete initialization.
"""

import ctypes
import ctypes.util
import os

import mujoco.egl
import rcs._core as _cxx
from mujoco.egl import GLContext

name = ctypes.util.find_library("EGL")
if name is None:
    msg = "libEGL not found"
    raise OSError(msg)

_egl = ctypes.CDLL(name, mode=os.RTLD_LOCAL | os.RTLD_NOW)

addr_make_current = ctypes.cast(_egl.eglMakeCurrent, ctypes.c_void_p).value

ctx = GLContext(max_width=3840, max_height=2160)

egl_display = mujoco.egl.EGL_DISPLAY.address
egl_context = ctx._context.address


def bootstrap():
    assert addr_make_current is not None
    _cxx.common._bootstrap_egl(addr_make_current, egl_display, egl_context)
