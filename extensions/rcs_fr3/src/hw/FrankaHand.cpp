#include "FrankaHand.h"

#include <franka/exception.h>
#include <franka/gripper.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

namespace rcs {
namespace hw {

FrankaHand::FrankaHand(const FHConfig& cfg) : gripper(cfg.ip), m_cfg(cfg) {
  this->m_reset();
}

FrankaHand::~FrankaHand() {
  try {
    this->m_stop();
  } catch (const franka::Exception& e) {
    std::cerr << "Exception in ~FrankaHand(): " << e.what() << std::endl;
  }
}

bool FrankaHand::set_config(const FHConfig& cfg) {
  franka::GripperState gripper_state = this->gripper.readOnce();
  if (gripper_state.max_width < cfg.grasping_width) {
    return false;
  }
  this->m_cfg = cfg;
  return true;
}

FHConfig* FrankaHand::get_config() {
  // copy config to heap
  FHConfig* cfg = new FHConfig();
  *cfg = this->m_cfg;
  return cfg;
}

FHState* FrankaHand::get_state() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  if (std::abs(gripper_state.max_width - this->m_cfg.grasping_width) > 0.01) {
    this->max_width = gripper_state.max_width - 0.001;
  }
  FHState* state = new FHState();
  state->width = gripper_state.width / gripper_state.max_width;
  state->is_grasped = gripper_state.is_grasped;
  state->temperature = gripper_state.temperature;
  state->max_unnormalized_width = this->max_width;
  state->last_commanded_width = this->last_commanded_width;
  state->bool_state = this->last_commanded_width > 0;
  state->is_moving = this->is_moving.load();
  return state;
}

void FrankaHand::execute_width_command(const WidthCommand& command) {
  if (command.force < 0.01) {
    this->gripper.move(command.width, this->m_cfg.speed);
  } else {
    this->gripper.grasp(command.width, this->m_cfg.speed,
                        command.force * this->m_cfg.force,
                        this->m_cfg.epsilon_inner, this->m_cfg.epsilon_outer);
  }
}

void FrankaHand::update_cached_width() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  this->cached_normalized_width.store(gripper_state.width /
                                      gripper_state.max_width);
}

void FrankaHand::run_width_commands(WidthCommand command) {
  while (true) {
    try {
      this->execute_width_command(command);
      this->update_cached_width();
    } catch (const franka::Exception& e) {
      std::cerr << "FrankaHand width command failed: " << e.what() << std::endl;
    }

    std::lock_guard<std::mutex> lock(this->width_command_mutex);
    if (this->pending_width_command.has_value()) {
      command = *this->pending_width_command;
      this->pending_width_command.reset();
      continue;
    }
    this->width_worker_active = false;
    this->is_moving.store(false);
    return;
  }
}

void FrankaHand::set_normalized_width(double width, double force) {
  if (width < 0 || width > 1 || force < 0 || force > 1) {
    throw std::invalid_argument("width and force must be between 0 and 1");
  }

  WidthCommand command{width * this->normalization_width, force};
  this->last_commanded_width = command.width;
  if (!this->m_cfg.async_control) {
    this->is_moving.store(true);
    try {
      this->execute_width_command(command);
      this->update_cached_width();
    } catch (...) {
      this->is_moving.store(false);
      throw;
    }
    this->is_moving.store(false);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(this->width_command_mutex);
    if (this->width_worker_active) {
      // A libfranka gripper command cannot be retargeted while it is running.
      // Retain only the newest request and execute it as soon as the current
      // command completes, without blocking the environment step.
      this->pending_width_command = command;
      return;
    }
  }

  // Reap a previously completed worker before replacing its std::thread.
  this->m_wait();
  {
    std::lock_guard<std::mutex> lock(this->width_command_mutex);
    this->width_worker_active = true;
    this->pending_width_command.reset();
  }
  this->is_moving.store(true);
  this->control_thread =
      std::thread(&FrankaHand::run_width_commands, this, command);
}

double FrankaHand::get_normalized_width() {
  if (this->m_cfg.async_control) {
    // readOnce() contends with an active libfranka move/grasp call and can
    // block until the motion finishes.  The worker refreshes this cache after
    // every completed command, so observations remain nonblocking.
    return this->cached_normalized_width.load();
  }
  this->update_cached_width();
  return this->cached_normalized_width.load();
}

void FrankaHand::m_stop() {
  {
    std::lock_guard<std::mutex> lock(this->width_command_mutex);
    this->pending_width_command.reset();
  }
  try {
    this->gripper.stop();
  } catch (const franka::Exception& e) {
    std::cerr << "FrankaHand::m_stop: " << e.what() << std::endl;
  }
  this->m_wait();
  {
    std::lock_guard<std::mutex> lock(this->width_command_mutex);
    this->width_worker_active = false;
  }
  this->is_moving.store(false);
}

void FrankaHand::m_wait() {
  if (this->control_thread.has_value()) {
    if (this->control_thread->joinable()) {
      this->control_thread->join();
    }
    this->control_thread.reset();
  }
}

void FrankaHand::m_reset() {
  this->m_stop();
  // open gripper
  franka::GripperState gripper_state = this->gripper.readOnce();
  this->normalization_width = gripper_state.max_width;
  this->max_width = gripper_state.max_width - 0.001;
  this->last_commanded_width = this->max_width;
  this->is_moving.store(true);
  this->gripper.move(this->max_width, this->m_cfg.speed);
  this->update_cached_width();
  this->is_moving.store(false);
}
void FrankaHand::reset() { this->m_reset(); }

bool FrankaHand::is_grasped() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  return gripper_state.is_grasped;
}

bool FrankaHand::homing() {
  // Do a homing in order to estimate the maximum
  // grasping width with the current fingers.
  this->is_moving.store(true);
  bool success = this->gripper.homing();
  this->update_cached_width();
  this->is_moving.store(false);
  return success;
}

void FrankaHand::grasp() {
  if (this->is_moving.load() || this->last_commanded_width == 0) {
    return;
  }
  this->last_commanded_width = 0;
  if (!this->m_cfg.async_control) {
    this->is_moving.store(true);
    this->gripper.grasp(0, this->m_cfg.speed, this->m_cfg.force, 1, 1);
    this->update_cached_width();
    this->is_moving.store(false);
    return;
  }
  this->m_wait();
  this->is_moving.store(true);
  this->control_thread = std::thread([this]() {
    try {
      this->gripper.grasp(0, this->m_cfg.speed, this->m_cfg.force, 1, 1);
      this->update_cached_width();
    } catch (const franka::CommandException& e) {
      std::cerr << "franka hand command exception ignored grasp" << std::endl;
    }
    this->is_moving.store(false);
  });
}

void FrankaHand::open() {
  if (this->is_moving.load() || this->last_commanded_width == this->max_width) {
    return;
  }
  this->last_commanded_width = this->max_width;
  if (!this->m_cfg.async_control) {
    this->is_moving.store(true);
    this->gripper.move(this->max_width, this->m_cfg.speed);
    this->update_cached_width();
    this->is_moving.store(false);
    return;
  }
  this->m_wait();
  this->is_moving.store(true);
  this->control_thread = std::thread([this]() {
    try {
      // perhaps we should use graps here
      this->gripper.move(this->max_width, this->m_cfg.speed);
      this->update_cached_width();
      // this->gripper.grasp(this->max_width, this->m_cfg.speed,
      // this->m_cfg.force, 1, 1);
    } catch (const franka::CommandException& e) {
      std::cerr << "franka hand command exception ignored open" << std::endl;
    }
    this->is_moving.store(false);
  });
}
void FrankaHand::shut() {
  if (this->is_moving.load() || this->last_commanded_width == 0) {
    return;
  }
  this->last_commanded_width = 0;
  if (!this->m_cfg.async_control) {
    this->is_moving.store(true);
    this->gripper.move(0, this->m_cfg.speed);
    this->update_cached_width();
    this->is_moving.store(false);
    return;
  }
  this->m_wait();
  this->is_moving.store(true);
  this->control_thread = std::thread([this]() {
    this->gripper.move(0, this->m_cfg.speed);
    this->update_cached_width();
    this->is_moving.store(false);
  });
}

void FrankaHand::close() { this->m_stop(); }
}  // namespace hw
}  // namespace rcs
