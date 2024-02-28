#include "FrankaHand.h"

#include <franka/gripper.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <tuple>

FrankaHand::FrankaHand(const std::string ip) : Gripper(), gripper(ip) {}

FrankaHand::~FrankaHand() {}

// Methods from Device
void FrankaHand::close() {}
void FrankaHand::open() {}
void FrankaHand::start() {
  // Do a homing in order to estimate the maximum
  // grasping width with the current fingers.
  gripper.homing();
}
void FrankaHand::stop() { gripper.stop(); }

bool FrankaHand::setParameters(double gw, double sp, double fr) {
  franka::GripperState gripper_state = gripper.readOnce();
  if (gripper_state.max_width < gw) {
    return false;
  }
  this->grasping_width = gw;
  this->speed = sp;
  this->force = fr;
  return true;
}

// Methods to read current state
std::tuple<double, double, bool> FrankaHand::getState() {
  franka::GripperState gripper_state = gripper.readOnce();
  return std::make_tuple(gripper_state.width, gripper_state.max_width,
                         gripper_state.is_grasped);
}

// Methods from Gripper
void FrankaHand::halt() {
  gripper.grasp(grasping_width, speed, force, 0.1, 0.1);
}
void FrankaHand::release() {
  franka::GripperState gripper_state = gripper.readOnce();
  gripper.move(gripper_state.max_width, speed);
}
void FrankaHand::shut() { gripper.move(grasping_width, speed); }
