#include "utils.h"

#include <EGL/egl.h>

#include <iostream>

namespace rcs {
namespace common {
static PFNEGLMAKECURRENTPROC g_makeCurrent = nullptr;
static EGLDisplay g_display = EGL_NO_DISPLAY;
static EGLSurface g_surface = EGL_NO_SURFACE;
static EGLContext g_context = EGL_NO_CONTEXT;

void bootstrap_egl(uintptr_t fn_addr, uintptr_t dpy, uintptr_t ctx) {
  g_makeCurrent = reinterpret_cast<PFNEGLMAKECURRENTPROC>(fn_addr);
  g_display = reinterpret_cast<EGLDisplay>(dpy);
  g_context = reinterpret_cast<EGLContext>(ctx);
}

void ensure_current() {
  if (!g_makeCurrent(g_display, g_surface, g_surface, g_context))
    throw std::runtime_error("eglMakeCurrent failed");
}
}  // namespace common
}  // namespace rcs
