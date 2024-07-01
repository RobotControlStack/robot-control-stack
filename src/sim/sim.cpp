#include "sim/sim.h"

#include <GLFW/glfw3.h>
#include <assert.h>

#include <chrono>
#include <functional>
#include <thread>

namespace rcs {
namespace sim {

void process_condition_callbacks(std::vector<ConditionCallback>& cbs,
                                 mjtNum time) {
  for (int i = 0; i < std::size(cbs); ++i) {
    mjtNum dt = time - cbs[i].last_call_timestamp;
    if (dt > cbs[i].seconds_between_calls) {
      cbs[i].last_return_value = cbs[i].cb();
    }
  }
}

bool get_last_return_value(ConditionCallback cb) {
  return cb.last_return_value;
}

Sim::Sim(mjModel* m, mjData* d) : m(m), d(d), renderer(m){};

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

bool Sim::invoke_condition_callbacks() {
  process_condition_callbacks(this->any_callbacks, this->d->time);
  process_condition_callbacks(this->all_callbacks, this->d->time);
  if (std::any_of(this->any_callbacks.begin(), this->any_callbacks.end(),
                  get_last_return_value)) {
    return true;
  }
  if (std::all_of(this->all_callbacks.begin(), this->all_callbacks.end(),
                  get_last_return_value)) {
    return true;
  }
  return false;
}

void Sim::invoke_rendering_callbacks() {
  for (size_t i = 0; i < std::size(this->rendering_callbacks); ++i) {
    RenderingCallback cb = this->rendering_callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      mjrContext* ctx = this->renderer.get_context(cb.id);
      cb.cb(*ctx, this->renderer.scene, this->renderer.opt);
    }
  }
}

void Sim::step_until_convergence() {
  /* Reset the condition callbacks */
  for (size_t i = 0; i < std::size(this->any_callbacks); ++i) {
    this->any_callbacks[i].last_return_value = false;
  }
  for (size_t i = 0; i < std::size(this->all_callbacks); ++i) {
    this->all_callbacks[i].last_return_value = false;
  }
  /* Step until all all_callbacks returned true or any any_callback returned
   * true */
  bool converged = false;
  while (not converged) {
    this->step(1);
    converged = invoke_condition_callbacks();
  };
}

void Sim::step(size_t k) {
  for (size_t i = 0; i < k; ++i) {
    mj_step1(this->m, this->d);
    this->invoke_callbacks();
    mj_step2(this->m, this->d);
    this->invoke_rendering_callbacks();
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
      ConditionCallback{cb, seconds_between_calls, 0.0, false});
}

void Sim::register_all_cb(std::function<bool(void)> cb,
                          mjtNum seconds_between_calls) {
  this->all_callbacks.push_back(
      ConditionCallback{cb, seconds_between_calls, 0.0, false});
}

void Sim::register_rendering_callback(
    std::function<void(mjrContext&, mjvScene&, mjvOption&)> cb,
    mjtNum seconds_between_calls, size_t width, size_t height, bool offscreen) {
  this->rendering_callbacks.push_back(RenderingCallback{
      .cb = cb,
      .id = this->renderer.register_context(width, height, offscreen),
      .seconds_between_calls = seconds_between_calls,
      .last_call_timestamp = 0});
}
}  // namespace sim
}  // namespace rcs
