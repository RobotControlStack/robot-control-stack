#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>
#include <stdexcept>

#include "sim/sim.h"
#include <iostream>
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
  this->ctxs = std::unordered_map<std::string, std::pair<GLFWwindow*, mjrContext*>>();
}

Renderer::~Renderer() {
  for (auto const& [id, ctx] : this->ctxs) {
    mjr_freeContext(ctx.second);
    glfwDestroyWindow(ctx.first);
  }
  mjv_freeScene(&this->scene);
}

void Renderer::register_context(const std::string& id, size_t width, size_t height, bool offscreen) {
  // create invisible window, single-buffered
  if (offscreen) {
    glfwWindowHint(GLFW_VISIBLE, 0);
  } else {
    glfwWindowHint(GLFW_VISIBLE, 1);
  }
  // In record.cc this is false, glfw says in practice you always want true.
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(width, height, "rcs", NULL, NULL);

  if (!window) {
    glfwTerminate();
    throw std::runtime_error("Could not create GLFW window");
  }
  glfwMakeContextCurrent(window);
  mjrContext* ctx = new mjrContext();
  mjr_defaultContext(ctx);
  mjr_makeContext(this->m, ctx, 200);
  if (offscreen) {
    mjr_setBuffer(mjFB_OFFSCREEN, ctx);
    mjr_resizeOffscreen(width, height, ctx);
    if (ctx->currentBuffer != mjFB_OFFSCREEN) {
      std::cout << "Warning: offscreen rendering not supported, using "
                   "default/window framebuffer"
                << std::endl;
    }
  } else {
    mjr_setBuffer(mjFB_WINDOW, ctx);
  }
  this->ctxs[id] = std::pair(window, ctx);
}

mjrContext* Renderer::get_context(const std::string& id) {
  glfwMakeContextCurrent(ctxs[id].first);
  return ctxs[id].second;
}
}  // namespace sim
}  // namespace rcs
