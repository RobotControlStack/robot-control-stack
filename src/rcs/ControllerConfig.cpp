#include "rcs/ControllerConfig.h"

#include <stdexcept>

#include "rcs/LinearInterpolator.h"

namespace rcs {
namespace common {

ControllerPair make_controller(const ControllerConfig& config,
                               double interpolation_time,
                               const JointLimits& joint_limits,
                               const std::optional<SafetyConfig>& safety,
                               double timestep) {
  auto joint_interpolator = [&] {
    return std::make_shared<LinearJointInterpolator>(interpolation_time);
  };
  auto cartesian_interpolator = [&] {
    return std::make_shared<LinearCartesianInterpolator>(interpolation_time);
  };
  ControllerPair controller;
  if (auto* c = std::get_if<JointPDConfig>(&config)) {
    if (c->kp.size() == 0 || c->kd.size() == 0) {
      throw std::invalid_argument("JointPDConfig needs kp and kd");
    }
    controller.joint =
        std::make_shared<JointPD>(c->kp, c->kd, joint_interpolator());
  } else if (auto* c = std::get_if<OSCConfig>(&config)) {
    controller.cartesian = std::make_shared<OSC>(*c, cartesian_interpolator());
  } else if (auto* c = std::get_if<FR3JointControllerConfig>(&config)) {
    controller.joint =
        std::make_shared<FR3JointController>(*c, joint_interpolator());
  } else if (auto* c = std::get_if<FR3OSCConfig>(&config)) {
    controller.cartesian =
        std::make_shared<FR3OSC>(*c, cartesian_interpolator());
  }
  if (safety.has_value()) {
    if (controller.joint) {
      controller.joint = std::make_shared<SafeJointController>(
          controller.joint, *safety, joint_limits, timestep);
    } else {
      controller.cartesian = std::make_shared<SafeCartesianController>(
          controller.cartesian, *safety, joint_limits, timestep);
    }
  }
  return controller;
}

}  // namespace common
}  // namespace rcs
