#include "rcs/SafetyStage.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace rcs {
namespace common {

SafetyStage::SafetyStage(const SafetyConfig& config,
                         const JointLimits& joint_limits, double timestep)
    : config(config), joint_limits(joint_limits), timestep(timestep) {
  if (!(timestep > 0.0)) {
    throw std::invalid_argument("timestep must be positive");
  }
  if (config.torque_limit.size() != 0 &&
      config.torque_limit.size() != joint_limits.cols()) {
    throw std::invalid_argument(
        "torque_limit needs one entry per joint of joint_limits");
  }
}

void SafetyStage::reset(const ControllerState& state) {
  if (state.q.size() != this->joint_limits.cols()) {
    throw std::invalid_argument("state and joint_limits differ in joints");
  }
  this->last_tau = VectorXd::Zero(state.q.size());
}

VectorXd SafetyStage::apply(const VectorXd& tau_in,
                            const ControllerState& state) {
  if (this->last_tau.size() != tau_in.size()) {
    throw std::logic_error("call reset() before apply()");
  }
  VectorXd tau = tau_in;
  const double margin = this->config.joint_limit_margin;
  const double max_step = this->config.max_torque_rate * this->timestep;
  for (Eigen::Index i = 0; i < tau.size(); i++) {
    if (this->joint_limits(1, i) - state.q[i] < margin && tau[i] > 0.)
      tau[i] = 0.;
    if (state.q[i] - this->joint_limits(0, i) < margin && tau[i] < 0.)
      tau[i] = 0.;
  }
  for (Eigen::Index i = 0; i < tau.size(); i++) {
    tau[i] = this->last_tau[i] +
             std::clamp(tau[i] - this->last_tau[i], -max_step, max_step);
    if (this->config.torque_limit.size() != 0) {
      tau[i] = std::clamp(tau[i], -this->config.torque_limit[i],
                          this->config.torque_limit[i]);
    }
  }
  this->last_tau = tau;
  return tau;
}

SafeJointController::SafeJointController(
    std::shared_ptr<JointController> controller, const SafetyConfig& config,
    const JointLimits& joint_limits, double timestep)
    : controller(std::move(controller)), stage(config, joint_limits, timestep) {
  if (!this->controller) {
    throw std::invalid_argument("controller must not be null");
  }
}

void SafeJointController::reset(const ControllerState& state) {
  this->controller->reset(state);
  this->stage.reset(state);
}

void SafeJointController::set_goal(const VectorXd& q_goal,
                                   std::optional<double> duration) {
  this->controller->set_goal(q_goal, duration);
}

VectorXd SafeJointController::step(const ControllerState& state) {
  return this->stage.apply(this->controller->step(state), state);
}

SafeCartesianController::SafeCartesianController(
    std::shared_ptr<CartesianController> controller, const SafetyConfig& config,
    const JointLimits& joint_limits, double timestep)
    : controller(std::move(controller)), stage(config, joint_limits, timestep) {
  if (!this->controller) {
    throw std::invalid_argument("controller must not be null");
  }
}

void SafeCartesianController::reset(const ControllerState& state) {
  this->controller->reset(state);
  this->stage.reset(state);
}

void SafeCartesianController::set_goal(const Pose& pose_goal,
                                       std::optional<double> duration) {
  this->controller->set_goal(pose_goal, duration);
}

VectorXd SafeCartesianController::step(const ControllerState& state) {
  return this->stage.apply(this->controller->step(state), state);
}

}  // namespace common
}  // namespace rcs
