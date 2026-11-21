#include "rcs/ControllerConfig.h"

#include <stdexcept>

#include "rcs/LinearInterpolator.h"

namespace rcs {
namespace common {

ControllerPair make_controller(const ControllerConfig& config,
                               double interpolation_time,
                               const JointLimits& joint_limits) {
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
    controller.cartesian =
        std::make_shared<OSC>(*c, cartesian_interpolator());
  } else if (auto* c = std::get_if<FR3JointControllerConfig>(&config)) {
    controller.joint = std::make_shared<FR3JointController>(
        *c, joint_interpolator(), joint_limits);
  } else if (auto* c = std::get_if<FR3OSCConfig>(&config)) {
    controller.cartesian = std::make_shared<FR3OSC>(
        *c, cartesian_interpolator(), joint_limits);
  }
  return controller;
}

}  // namespace common
}  // namespace rcs
