#include "sim/sim.h"

#include <assert.h>

#include <functional>
#include <stop_token>

namespace rcs {
namespace sim {

Sim::Sim(mjModel* m, mjData* d) : m(m), d(d){};

bool Sim::set_config(const Config& cfg) {
  if (not cfg.async and cfg.realtime) return false;
  if (cfg.async) {
    /* Not implemented :) */
    return false;
  }
  this->cfg = cfg;
  return true;
}
Config Sim::get_config() { return this->cfg; }

void Sim::step_until_convergence() {
  bool converged = false;
  while (not converged) {
    // TODO: make sure it makes sense with the step split in two like that.
    mj_step1(this->m, this->d);
    for (int i = 0; i < std::size(this->callbacks); ++i) {
      this->callbacks[i]();
    }
    mj_step2(this->m, this->d);
    converged = true;
    for (int i = 0; i < std::size(this->convergence_callbacks); ++i) {
      if (not this->convergence_callbacks[i]()) {
        converged = false;
      }
    }
  };
}

void Sim::step(size_t k) {
  for (size_t i = 0; i < k; ++i) {
    mj_step1(this->m, this->d);
    for (int i = 0; i < std::size(this->callbacks); ++i) {
      this->callbacks[i]();
    }
    mj_step2(this->m, this->d);
  }
}

void Sim::register_cb(std::function<void(void)> cb) {
  this->callbacks.push_back(cb);
}

void Sim::register_convergence_cb(std::function<bool(void)> cb) {
  this->convergence_callbacks.push_back(cb);
}
}  // namespace sim
}  // namespace rcs
