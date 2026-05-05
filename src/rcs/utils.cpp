#include "rcs/utils.h"

#include <EGL/egl.h>

#include <iostream>

namespace rcs {
namespace common {
enum class RenderBackend {
  none,
  egl,
  current_context,
};

static PFNEGLMAKECURRENTPROC g_makeCurrent = nullptr;
static EGLDisplay g_display = EGL_NO_DISPLAY;
static EGLSurface g_surface = EGL_NO_SURFACE;
static EGLContext g_context = EGL_NO_CONTEXT;
static RenderBackend g_render_backend = RenderBackend::none;

void bootstrap_egl(uintptr_t fn_addr, uintptr_t dpy, uintptr_t ctx) {
  g_makeCurrent = reinterpret_cast<PFNEGLMAKECURRENTPROC>(fn_addr);
  g_display = reinterpret_cast<EGLDisplay>(dpy);
  g_context = reinterpret_cast<EGLContext>(ctx);
  g_render_backend = RenderBackend::egl;
}

void bootstrap_gl_context() {
  g_makeCurrent = nullptr;
  g_display = EGL_NO_DISPLAY;
  g_context = EGL_NO_CONTEXT;
  g_render_backend = RenderBackend::current_context;
}

void ensure_current() {
  if (g_render_backend == RenderBackend::current_context) {
    return;
  }
  if (g_render_backend != RenderBackend::egl || g_makeCurrent == nullptr) {
    throw std::runtime_error(
        "No rendering context backend initialized. "
        "Call the Python rendering bootstrap before using simulation cameras.");
  }
  if (!g_makeCurrent(g_display, g_surface, g_surface, g_context)) {
    throw std::runtime_error("eglMakeCurrent failed");
  }
}
}  // namespace common
}  // namespace rcs
