#include "sim/sim.h"

#include <GLFW/glfw3.h>
#include <assert.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

namespace rcs {
namespace sim {

void process_condition_callbacks(std::vector<ConditionCallback>& cbs,
                                 mjtNum time) {
  for (int i = 0; i < std::size(cbs); ++i) {
    mjtNum dt = time - cbs[i].last_call_timestamp;
    if (dt > cbs[i].seconds_between_calls) {
      cbs[i].last_return_value = cbs[i].cb();
      cbs[i].last_call_timestamp = time;
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
    Callback& cb = this->callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      cb.cb();
      cb.last_call_timestamp = this->d->time;
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
  // bool scene_updated = false;
  for (size_t i = 0; i < std::size(this->rendering_callbacks); ++i) {
    RenderingCallback& cb = this->rendering_callbacks[i];
    mjtNum dt = this->d->time - cb.last_call_timestamp;
    if (dt > cb.seconds_between_calls) {
      // if (!scene_updated) {
      //   // update scene once for all cameras
      //   mjv_updateScene(this->m, this->d, &this->renderer.opt, NULL, NULL,
      //   mjCAT_ALL,
      //                   &this->renderer.scene);
      //   scene_updated = true;
      // }
      mjrContext* ctx = this->renderer.get_context(cb.id);
      cb.cb(cb.id, *ctx, this->renderer.scene, this->renderer.opt);
      cb.last_call_timestamp = this->d->time;
    }
  }
}

bool Sim::is_converged() { return this->converged; }
void Sim::step_until_convergence() {
  this->convergence_steps = 0;
  this->converged = false;
  /* Reset the condition callbacks */
  for (size_t i = 0; i < std::size(this->any_callbacks); ++i) {
    this->any_callbacks[i].last_return_value = false;
  }
  for (size_t i = 0; i < std::size(this->all_callbacks); ++i) {
    this->all_callbacks[i].last_return_value = false;
  }
  /* Step until all all_callbacks returned true or any any_callback returned
   * true */
  while (not this->converged and
         (this->cfg.max_convergence_steps == -1 or
          this->convergence_steps < this->cfg.max_convergence_steps)) {
    this->step(1);
    this->convergence_steps++;
    this->converged = invoke_condition_callbacks();
  };
  // if (this->convergence_steps == this->cfg.max_convergence_steps) {
  //   std::cerr << "WARNING: Max convergence steps reached!" << std::endl;
  // }
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

void Sim::reset() {
  mj_resetData(this->m, this->d);
  this->reset_callbacks();
}

void Sim::reset_callbacks() {
  for (size_t i = 0; i < std::size(this->callbacks); ++i) {
    this->callbacks[i].last_call_timestamp = 0;
  }
  for (size_t i = 0; i < std::size(this->any_callbacks); ++i) {
    this->any_callbacks[i].last_call_timestamp = 0;
  }
  for (size_t i = 0; i < std::size(this->all_callbacks); ++i) {
    this->all_callbacks[i].last_call_timestamp = 0;
  }
  for (size_t i = 0; i < std::size(this->rendering_callbacks); ++i) {
    // this is negative so that we will directly render the cameras
    // in the first step
    this->rendering_callbacks[i].last_call_timestamp =
        -this->rendering_callbacks[i].seconds_between_calls;
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
    std::function<void(const std::string&, mjrContext&, mjvScene&, mjvOption&)>
        cb,
    const std::string& id, int frame_rate, size_t width, size_t height) {
  this->renderer.register_context(id, width, height);
  // in case frame_rate is zero, rendering needs to be triggered
  // manually
  if (frame_rate != 0) {
    this->rendering_callbacks.push_back(
        RenderingCallback{.cb = cb,
                          .id = id,
                          .seconds_between_calls = 1.0 / frame_rate,
                          // this is negative so that we will directly render
                          // the cameras in the first step
                          .last_call_timestamp = -1.0 / frame_rate});
  }
}

void Sim::start_gui_server(const std::string& id) {
  if (this->gui.has_value()) {
    throw std::runtime_error("Start gui server should be called only once.");
  }
  this->gui.emplace(this->m, this->d, id);
  this->register_cb(
      std::bind(&GuiServer::update_mjdata_callback, &this->gui.value()), 0);
}
// TODO: when stop_gui_server is called, the callback still exists but now
// points to an non existing gui
void Sim::stop_gui_server() { this->gui.reset(); }
}  // namespace sim
}  // namespace rcs
