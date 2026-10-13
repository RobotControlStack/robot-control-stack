#include "rcs/utils.h"

#include <EGL/egl.h>

#include <iostream>

namespace rcs {
namespace common {
enum class RenderBackend { none, egl, current_context };

static RenderBackend g_backend = RenderBackend::none;
static PFNEGLMAKECURRENTPROC g_makeCurrent = nullptr;
static EGLDisplay g_display = EGL_NO_DISPLAY;
static EGLSurface g_surface = EGL_NO_SURFACE;
static EGLContext g_context = EGL_NO_CONTEXT;

void bootstrap_egl_context(uintptr_t fn_addr, uintptr_t dpy, uintptr_t ctx) {
  g_makeCurrent = reinterpret_cast<PFNEGLMAKECURRENTPROC>(fn_addr);
  g_display = reinterpret_cast<EGLDisplay>(dpy);
  g_context = reinterpret_cast<EGLContext>(ctx);
  g_backend = RenderBackend::egl;
}

void bootstrap_gl_context() { g_backend = RenderBackend::current_context; }

void ensure_current() {
  switch (g_backend) {
    case RenderBackend::current_context:
      return;
    case RenderBackend::egl:
      if (!g_makeCurrent(g_display, g_surface, g_surface, g_context))
        throw std::runtime_error("eglMakeCurrent failed");
      return;
    case RenderBackend::none:
    default:
      throw std::runtime_error(
          "Rendering was requested, but no render backend was bootstrapped. "
          "This usually means libEGL or the MuJoCo GL context is unavailable. "
          "Run without cameras/viewers if you do not need rendering, or "
          "install "
          "the required system EGL/OpenGL runtime libraries.");
  }
}
}  // namespace common
}  // namespace rcs
