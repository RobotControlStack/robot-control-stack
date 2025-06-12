#include <common/utils.h>
#include <mujoco/mjrender.h>
#include <mujoco/mujoco.h>

#include <iostream>
#include <stdexcept>

#include "sim/sim.h"
namespace rcs {
namespace sim {

Renderer::Renderer(mjModel* m) : m{m} {
  mjv_defaultOption(&this->opt);
  mjv_defaultScene(&this->scene);
  size_t max_geoms = 2000;
  mjv_makeScene(this->m, &this->scene, max_geoms);
  this->ctxs = std::unordered_map<std::string, mjrContext*>();
}

Renderer::~Renderer() {
  for (auto const& [id, ctx] : this->ctxs) {
    mjr_freeContext(ctx);
  }
  mjv_freeScene(&this->scene);
}

void Renderer::register_context(const std::string& id, size_t width,
                                size_t height) {
  common::ensure_current();
  mjrContext* ctx = new mjrContext();
  mjr_defaultContext(ctx);
  mjr_makeContext(this->m, ctx, 200);
  mjr_setBuffer(mjFB_OFFSCREEN, ctx);
  mjr_resizeOffscreen(width, height, ctx);
  if (ctx->currentBuffer != mjFB_OFFSCREEN) {
    std::cout << "Warning: offscreen rendering not supported, using "
                 "default/window framebuffer"
              << std::endl;
  }
  this->ctxs[id] = ctx;
}

mjrContext* Renderer::get_context(const std::string& id) {
  common::ensure_current();
  return this->ctxs[id];
}
}  // namespace sim
}  // namespace rcs
