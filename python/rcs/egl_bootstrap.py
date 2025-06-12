import ctypes
import ctypes.util
import os
import platform

import mujoco.egl
import rcs._core as _cxx
from mujoco.egl import GLContext
from mujoco.egl import egl_ext as EGL

name = ctypes.util.find_library("EGL")
if name is None:
    raise OSError("libEGL not found")

if platform.system() == "Windows":
    _egl = ctypes.WinDLL(name)
else:
    _egl = ctypes.CDLL(name, mode=os.RTLD_LOCAL | os.RTLD_NOW)

addr_make_current = ctypes.cast(
    _egl.eglMakeCurrent, ctypes.c_void_p
).value

ctx = GLContext(max_width=3840, max_height=2160)

egl_display = mujoco.egl.EGL_DISPLAY.address
egl_context = ctx._context.address

_cxx.common._bootstrap_egl(addr_make_current, egl_display, egl_context)
