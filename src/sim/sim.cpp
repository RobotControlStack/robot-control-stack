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

bool Sim::invoke_any_callbacks() {
  bool any = false;
  bool callback_called = false;
  for (int i = 0; i < std::size(this->any_callbacks); ++i) {
    ConditionCallback cb = this->any_callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      if (cb.cb()) {
        any = true;
      }
    }
  }
  return callback_called and any;
}

bool Sim::invoke_all_callbacks() {
  bool all = true;
  bool callback_called = false;
  for (int i = 0; i < std::size(this->all_callbacks); ++i) {
    ConditionCallback cb = this->all_callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      if (not cb.cb()) {
        all = false;
      }
      callback_called = true;
    }
  }
  return callback_called and all;
}

void Sim::step_until_convergence() {
  bool any = false;
  bool all = false;
  bool converged = false;
  while (not converged) {
    this->step(1);
    any = this->invoke_any_callbacks();
    all = this->invoke_all_callbacks();
    converged = any or all;
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

void Sim::register_any_cb(std::function<bool(void)> cb,
                          mjtNum seconds_between_calls) {
  this->any_callbacks.push_back(
      ConditionCallback{cb, seconds_between_calls, 0.0});
}

void Sim::register_all_cb(std::function<bool(void)> cb,
                          mjtNum seconds_between_calls) {
  this->all_callbacks.push_back(
      ConditionCallback{cb, seconds_between_calls, 0.0});
}
}  // namespace sim
}  // namespace rcs
