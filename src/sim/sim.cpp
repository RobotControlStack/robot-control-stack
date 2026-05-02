#include "sim/sim.h"

#include <assert.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
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

int Sim::get_joint_qpos_size(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 7;
    case mjJNT_BALL:
      return 4;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
    default:
      throw std::runtime_error("Unsupported MuJoCo joint type for qpos size.");
  }
}

int Sim::get_joint_qvel_size(int joint_type) {
  switch (joint_type) {
    case mjJNT_FREE:
      return 6;
    case mjJNT_BALL:
      return 3;
    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      return 1;
    default:
      throw std::runtime_error("Unsupported MuJoCo joint type for qvel size.");
  }
}

void Sim::init_dynamic_joint_specs() {
  this->dynamic_joint_specs.clear();
  this->dynamic_joint_name_to_index.clear();

  for (int joint_id = 0; joint_id < this->m->njnt; ++joint_id) {
    const char* joint_name = mj_id2name(this->m, mjOBJ_JOINT, joint_id);
    if (joint_name == nullptr || joint_name[0] == '\0') {
      std::ostringstream msg;
      msg << "Dynamic joint state requires all joints to be named. Joint id "
          << joint_id << " is unnamed.";
      throw std::runtime_error(msg.str());
    }

    DynamicJointSpec spec{
        .name = joint_name,
        .type = this->m->jnt_type[joint_id],
        .qpos_adr = this->m->jnt_qposadr[joint_id],
        .qvel_adr = this->m->jnt_dofadr[joint_id],
        .qpos_size = get_joint_qpos_size(this->m->jnt_type[joint_id]),
        .qvel_size = get_joint_qvel_size(this->m->jnt_type[joint_id]),
    };

    this->dynamic_joint_name_to_index[spec.name] =
        this->dynamic_joint_specs.size();
    this->dynamic_joint_specs.push_back(spec);
  }
}

Sim::Sim(mjModel* m, mjData* d) : m(m), d(d), renderer(m) {
  this->init_dynamic_joint_specs();
};

bool Sim::set_config(const SimConfig& cfg) {
  this->cfg = cfg;
  return true;
}

SimConfig Sim::get_config() { return this->cfg; }

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
  if (this->convergence_steps == this->cfg.max_convergence_steps) {
    std::cerr << "WARNING: Max convergence steps reached!" << std::endl;
  }
}

void Sim::step(size_t k) {
  for (size_t i = 0; i < k; ++i) {
    mj_step1(this->m, this->d);
    this->invoke_callbacks();
    mj_step2(this->m, this->d);
    this->invoke_rendering_callbacks();
  }
}

void Sim::reset() {
  mj_resetData(this->m, this->d);
  this->reset_callbacks();
}

DynamicJointSchema Sim::get_dynamic_joint_schema() const {
  DynamicJointSchema schema;
  schema.joint_names.reserve(this->dynamic_joint_specs.size());
  schema.joint_types.reserve(this->dynamic_joint_specs.size());
  schema.qpos_sizes.reserve(this->dynamic_joint_specs.size());
  schema.qvel_sizes.reserve(this->dynamic_joint_specs.size());

  for (const DynamicJointSpec& spec : this->dynamic_joint_specs) {
    schema.joint_names.push_back(spec.name);
    schema.joint_types.push_back(spec.type);
    schema.qpos_sizes.push_back(spec.qpos_size);
    schema.qvel_sizes.push_back(spec.qvel_size);
  }
  return schema;
}

DynamicJointState Sim::get_dynamic_joint_state() const {
  DynamicJointState state;
  int total_qpos = 0;
  int total_qvel = 0;
  for (const DynamicJointSpec& spec : this->dynamic_joint_specs) {
    total_qpos += spec.qpos_size;
    total_qvel += spec.qvel_size;
  }

  state.qpos = rcs::common::VectorXd(total_qpos);
  state.qvel = rcs::common::VectorXd(total_qvel);

  int qpos_offset = 0;
  int qvel_offset = 0;
  for (const DynamicJointSpec& spec : this->dynamic_joint_specs) {
    for (int i = 0; i < spec.qpos_size; ++i) {
      state.qpos[qpos_offset + i] = this->d->qpos[spec.qpos_adr + i];
    }
    for (int i = 0; i < spec.qvel_size; ++i) {
      state.qvel[qvel_offset + i] = this->d->qvel[spec.qvel_adr + i];
    }
    qpos_offset += spec.qpos_size;
    qvel_offset += spec.qvel_size;
  }

  return state;
}

void Sim::set_dynamic_joint_state(const DynamicJointSchema& schema,
                                  const DynamicJointState& state) {
  size_t joint_count = schema.joint_names.size();
  if (schema.joint_types.size() != joint_count ||
      schema.qpos_sizes.size() != joint_count ||
      schema.qvel_sizes.size() != joint_count) {
    throw std::invalid_argument(
        "Dynamic joint schema fields must all have the same length.");
  }

  int expected_qpos_size = std::accumulate(schema.qpos_sizes.begin(),
                                           schema.qpos_sizes.end(), 0);
  int expected_qvel_size = std::accumulate(schema.qvel_sizes.begin(),
                                           schema.qvel_sizes.end(), 0);
  if (state.qpos.size() != expected_qpos_size) {
    std::ostringstream msg;
    msg << "Dynamic joint qpos size mismatch. Expected "
        << expected_qpos_size << ", got " << state.qpos.size() << ".";
    throw std::invalid_argument(msg.str());
  }
  if (state.qvel.size() != expected_qvel_size) {
    std::ostringstream msg;
    msg << "Dynamic joint qvel size mismatch. Expected "
        << expected_qvel_size << ", got " << state.qvel.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  std::vector<bool> matched_target_joints(this->dynamic_joint_specs.size(),
                                          false);
  int qpos_offset = 0;
  int qvel_offset = 0;
  for (size_t i = 0; i < joint_count; ++i) {
    auto spec_iter =
        this->dynamic_joint_name_to_index.find(schema.joint_names[i]);
    if (spec_iter == this->dynamic_joint_name_to_index.end()) {
      std::cerr << "WARNING: Recorded dynamic joint '" << schema.joint_names[i]
                << "' is missing in the replay model. Skipping it."
                << std::endl;
      qpos_offset += schema.qpos_sizes[i];
      qvel_offset += schema.qvel_sizes[i];
      continue;
    }

    const DynamicJointSpec& target_spec =
        this->dynamic_joint_specs[spec_iter->second];
    matched_target_joints[spec_iter->second] = true;
    if (target_spec.type != schema.joint_types[i] ||
        target_spec.qpos_size != schema.qpos_sizes[i] ||
        target_spec.qvel_size != schema.qvel_sizes[i]) {
      std::ostringstream msg;
      msg << "Dynamic joint schema mismatch for joint '"
          << schema.joint_names[i] << "': expected type=" << target_spec.type
          << ", qpos_size=" << target_spec.qpos_size
          << ", qvel_size=" << target_spec.qvel_size << " but got type="
          << schema.joint_types[i] << ", qpos_size=" << schema.qpos_sizes[i]
          << ", qvel_size=" << schema.qvel_sizes[i] << ".";
      throw std::invalid_argument(msg.str());
    }

    for (int j = 0; j < target_spec.qpos_size; ++j) {
      this->d->qpos[target_spec.qpos_adr + j] = state.qpos[qpos_offset + j];
    }
    for (int j = 0; j < target_spec.qvel_size; ++j) {
      this->d->qvel[target_spec.qvel_adr + j] = state.qvel[qvel_offset + j];
    }

    qpos_offset += schema.qpos_sizes[i];
    qvel_offset += schema.qvel_sizes[i];
  }

  for (size_t i = 0; i < this->dynamic_joint_specs.size(); ++i) {
    if (!matched_target_joints[i]) {
      std::cerr << "WARNING: Replay model dynamic joint '"
                << this->dynamic_joint_specs[i].name
                << "' is missing in the recorded schema. Leaving it at its "
                   "current value."
                << std::endl;
    }
  }

  mj_forward(this->m, this->d);
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
