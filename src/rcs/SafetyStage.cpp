#include "rcs/SafetyStage.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace rcs {
namespace common {

SafetyStage::SafetyStage(const SafetyConfig& config,
                         const JointLimits& joint_limits, double timestep)
    : config(config), joint_limits(joint_limits), timestep(timestep) {
  if (!(timestep > 0.0)) {
    throw std::invalid_argument("timestep must be positive");
  }
  const Eigen::Index n = joint_limits.cols();
  for (const auto& [name, v] : {std::pair<const char*, const VectorXd&>{
                                    "torque_limit", config.torque_limit},
                                {"max_velocity", config.max_velocity},
                                {"deceleration", config.deceleration},
                                {"velocity_offset", config.velocity_offset},
                                {"limit_damping", config.limit_damping},
                                {"spring_torque", config.spring_torque}}) {
    if (v.size() != 0 && v.size() != n) {
      throw std::invalid_argument(std::string(name) +
                                  " needs one entry per joint of joint_limits");
    }
  }
  if (config.limit_damping.size() != 0 &&
      (config.max_velocity.size() == 0 || config.deceleration.size() == 0)) {
    throw std::invalid_argument(
        "limit_damping needs max_velocity and deceleration");
  }
  if (config.spring_torque.size() != 0 && !(config.spring_range > 0.0)) {
    throw std::invalid_argument("spring_range must be positive");
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
  const double max_step = this->config.max_torque_rate * this->timestep;
  const SafetyConfig& c = this->config;
  for (Eigen::Index i = 0; i < tau.size(); i++) {
    const double offset = c.velocity_offset.size() ? c.velocity_offset[i] : 0.0;
    // distances to the upper and the lower limit
    const double d_upper = this->joint_limits(1, i) - state.q[i];
    const double d_lower = state.q[i] - this->joint_limits(0, i);
    if (c.limit_damping.size()) {
      auto allowed = [&](double d) {
        return c.velocity_margin *
               std::min(c.max_velocity[i],
                        std::max(0.0, std::sqrt(2.0 * c.deceleration[i] *
                                                std::max(d, 0.0)) -
                                          offset));
      };
      const double towards_upper = state.dq[i] - allowed(d_upper);
      const double towards_lower = -state.dq[i] - allowed(d_lower);
      if (towards_upper > 0.0) tau[i] -= c.limit_damping[i] * towards_upper;
      if (towards_lower > 0.0) tau[i] += c.limit_damping[i] * towards_lower;
    }
    if (c.spring_torque.size()) {
      const double d0 = c.deceleration.size() && c.deceleration[i] > 0.0
                            ? offset * offset / (2.0 * c.deceleration[i])
                            : 0.0;
      // how deep the joint is in the zone in front of the zero-velocity point
      auto depth = [&](double d) {
        return std::clamp((c.spring_range - (d - d0)) / c.spring_range, 0.0,
                          1.0);
      };
      // the controller's torque towards the limit fades out over the zone, so
      // that the spring always wins at the zero-velocity point
      if (tau_in[i] > 0.0) tau[i] -= tau_in[i] * depth(d_upper);
      if (tau_in[i] < 0.0) tau[i] -= tau_in[i] * depth(d_lower);
      tau[i] -= c.spring_torque[i] * depth(d_upper);
      tau[i] += c.spring_torque[i] * depth(d_lower);
    }
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
