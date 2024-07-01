#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>
#include <stdexcept>

#include "sim/sim.h"
namespace rcs {
namespace sim {

Renderer::Renderer(mjModel* m) : m{m} {
  if (!glfwInit()) {
    throw std::runtime_error("Could not initialize GLFW");
  }
  mjv_defaultOption(&this->opt);
  mjv_defaultScene(&this->scene);
  size_t max_geoms = 2000;
  mjv_makeScene(this->m, &this->scene, max_geoms);
}

Renderer::~Renderer() {
  for (size_t i = 0; i < std::size(this->ctxs); ++i) {
    mjr_freeContext(&this->ctxs[i].second);
  }
}

size_t Renderer::register_context(size_t width, size_t height, bool offscreen) {
  // create invisible window, single-buffered
  if (offscreen) {
    glfwWindowHint(GLFW_VISIBLE, 0);
  } else {
    glfwWindowHint(GLFW_VISIBLE, 1);
  }
  // In record.cc this is false, glfw says in practice you always want true.
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(width, height, "rcs", NULL, NULL);
  glfwMakeContextCurrent(window);
  mjrContext ctx;
  mjr_makeContext(this->m, &ctx, 200);
  if (offscreen) {
    mjr_setBuffer(mjFB_OFFSCREEN, &ctx);
  } else {
    mjr_setBuffer(mjFB_WINDOW, &ctx);
  }
  this->ctxs.push_back(std::pair(window, ctx));
  return std::size(this->ctxs) - 1;
}

mjrContext& Renderer::get_context(size_t id) {
  glfwMakeContextCurrent(ctxs[id].first);
  return ctxs[id].second;
}
}  // namespace sim
}  // namespace rcs
