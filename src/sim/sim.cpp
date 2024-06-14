#include "sim/sim.h"

#include <assert.h>

#include <chrono>
#include <functional>
#include <thread>

namespace rcs {
namespace sim {

Sim::Sim(mjModel* m, mjData* d) : m(m), d(d){};

bool Sim::set_config(const Config& cfg) {
  if (cfg.async) {
    /* Not implemented :) */
    return false;
  }
  this->cfg = cfg;
  return true;
}

Config Sim::get_config() { return this->cfg; }

void Sim::invoke_callbacks() {
  for (int i = 0; i < std::size(this->callbacks); ++i) {
    Callback cb = this->callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      cb.cb();
    }
  }
}

bool Sim::invoke_convergence_callbacks() {
  bool converged = true;
  for (int i = 0; i < std::size(this->convergence_callbacks); ++i) {
    ConvergenceCallback cb = this->convergence_callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      if (not cb.cb()) {
        converged = false;
      }
    }
  }
  return converged;
}

void Sim::step_until_convergence() {
  bool converged = false;
  while (not converged) {
    mj_step1(this->m, this->d);
    this->invoke_callbacks();
    mj_step2(this->m, this->d);
    this->invoke_convergence_callbacks();
  };
}

void Sim::step(size_t k) {
  for (size_t i = 0; i < k; ++i) {
    mj_step1(this->m, this->d);
    this->invoke_callbacks();
    mj_step2(this->m, this->d);
    if (this->cfg.realtime) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void Sim::register_cb(std::function<void(void)> cb,
                      mjtNum seconds_between_calls) {
  this->callbacks.push_back(Callback{cb, seconds_between_calls, 0.0});
}

void Sim::register_convergence_cb(std::function<bool(void)> cb,
                                  mjtNum seconds_between_calls) {
  this->convergence_callbacks.push_back(
      ConvergenceCallback{cb, seconds_between_calls, 0.0});
}
}  // namespace sim
}  // namespace rcs
