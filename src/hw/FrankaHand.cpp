#include "FrankaHand.h"

#include <franka/gripper.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <tuple>

namespace rcs {
namespace hw {

FrankaHand::FrankaHand(const std::string &ip,
                       const std::optional<FHConfig> &cfg)
    : gripper(ip) {
  if (cfg.has_value()) {
    this->cfg = cfg.value();
  }  // else default constructor
}

FrankaHand::~FrankaHand() {}

bool FrankaHand::set_parameters(const common::GConfig &cfg) {
  auto fhcfg = dynamic_cast<const FHConfig &>(cfg);

  franka::GripperState gripper_state = gripper.readOnce();
  if (gripper_state.max_width < fhcfg.grasping_width) {
    return false;
  }
  this->cfg = fhcfg;
  return true;
}

FHConfig *FrankaHand::get_parameters() {
  // copy config to heap
  FHConfig *cfg = new FHConfig();
  *cfg = this->cfg;
  return cfg;
}

FHState *FrankaHand::get_state() {
  franka::GripperState gripper_state = gripper.readOnce();
  FHState *state = new FHState();
  state->width = gripper_state.width;
  state->is_grasped = gripper_state.is_grasped;
  state->temperature = gripper_state.temperature;
  return state;
}

bool FrankaHand::homing() {
  // Do a homing in order to estimate the maximum
  // grasping width with the current fingers.
  return gripper.homing();
}

bool FrankaHand::grasp() {
  return gripper.grasp(this->cfg.grasping_width, this->cfg.speed,
                       this->cfg.force, this->cfg.epsilon_inner,
                       this->cfg.epsilon_outer);
}

void FrankaHand::release() {
  franka::GripperState gripper_state = gripper.readOnce();
  gripper.move(gripper_state.max_width, this->cfg.speed);
}
void FrankaHand::shut() { gripper.move(0, this->cfg.speed); }
}  // namespace hw
}  // namespace rcs