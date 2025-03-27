#include "FrankaHand.h"

#include <franka/gripper.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <tuple>

namespace rcs {
namespace hw {

FrankaHand::FrankaHand(const std::string &ip, const FHConfig &cfg)
    : gripper(ip), cfg{} {
  this->cfg = cfg;
  this->m_reset();
}

FrankaHand::~FrankaHand() {}

bool FrankaHand::set_parameters(const FHConfig &cfg) {
  franka::GripperState gripper_state = this->gripper.readOnce();
  if (gripper_state.max_width < cfg.grasping_width) {
    return false;
  }
  this->cfg = cfg;
  return true;
}

FHConfig *FrankaHand::get_parameters() {
  // copy config to heap
  FHConfig *cfg = new FHConfig();
  *cfg = this->cfg;
  return cfg;
}

FHState *FrankaHand::get_state() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  if (std::abs(gripper_state.max_width - this->cfg.grasping_width) > 0.01) {
    this->max_width = gripper_state.max_width - 0.001;
  }
  FHState *state = new FHState();
  state->width = gripper_state.width / gripper_state.max_width;
  state->is_grasped = gripper_state.is_grasped;
  state->temperature = gripper_state.temperature;
  state->max_unnormalized_width = this->max_width;
  state->last_commanded_width = this->last_commanded_width;
  return state;
}

void FrankaHand::set_normalized_width(double width, double force) {
  if (width < 0 || width > 1 || force < 0) {
    throw std::invalid_argument(
        "width must be between 0 and 1, force must be positive");
  }
  franka::GripperState gripper_state = this->gripper.readOnce();
  width = width * gripper_state.max_width;
  this->last_commanded_width = width;
  if (force < 0.01) {
    this->gripper.move(width, this->cfg.speed);
  } else {
    this->gripper.grasp(width, this->cfg.speed, force, this->cfg.epsilon_inner,
                  this->cfg.epsilon_outer);
  }
}
double FrankaHand::get_normalized_width() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  return gripper_state.width / gripper_state.max_width;
}

void FrankaHand::m_stop(){
  this->gripper.stop();
  if (this->control_thread.has_value()) {
    this->control_thread->join();
    this->control_thread.reset();
  }

}

void FrankaHand::m_reset() {
  this->m_stop();
  // open gripper
  franka::GripperState gripper_state = this->gripper.readOnce();
  this->max_width = gripper_state.max_width - 0.001;
  this->last_commanded_width = this->max_width;
  this->gripper.move(this->max_width, this->cfg.speed);
}
void FrankaHand::reset() { this->m_reset(); }

bool FrankaHand::is_grasped() {
  franka::GripperState gripper_state = this->gripper.readOnce();
  return gripper_state.is_grasped;
}

bool FrankaHand::homing() {
  // Do a homing in order to estimate the maximum
  // grasping width with the current fingers.
  return this->gripper.homing();
}

void FrankaHand::grasp() {
  this->last_commanded_width = 0;
  if (!this->cfg.async_control) {
    this->gripper.grasp(0, this->cfg.speed, this->cfg.force, 1, 1);
    return;
  }
  this->m_stop();
  this->control_thread = std::thread([&](){this->gripper.grasp(0, this->cfg.speed, this->cfg.force, 1, 1);});
}

void FrankaHand::open() {
  this->last_commanded_width = this->max_width;
  if (!this->cfg.async_control) {
    this->gripper.move(this->max_width, this->cfg.speed);
    return;
  }
  this->m_stop();
  this->control_thread = std::thread([&](){this->gripper.move(this->max_width, this->cfg.speed);});
}
void FrankaHand::shut() {
  this->last_commanded_width = 0;
  if (!this->cfg.async_control) {
    this->gripper.move(0, this->cfg.speed);
    return;
  }
  this->m_stop();
  this->control_thread = std::thread([&](){this->gripper.move(0, this->cfg.speed);});
}
}  // namespace hw
}  // namespace rcs