#ifndef RCS_CONTROLLER_CONFIG_H
#define RCS_CONTROLLER_CONFIG_H

#include <memory>
#include <variant>

#include "Controller.h"
#include "Controllers.h"
#include "FR3Controllers.h"
#include "SafetyStage.h"
#include "utils.h"

namespace rcs {
namespace common {

struct JointPDConfig {
  // one gain per joint, no defaults as they depend on the robot
  VectorXd kp;
  VectorXd kd;
  // damp towards the setpoint velocity instead of zero, see JointPD
  bool velocity_feedforward = false;
};

// A controller, identified by its config. RobotConfig::controller holds one;
// none means the robot's native control. The type of the config selects the
// controller; JointPDConfig and FR3JointControllerConfig give joint
// controllers, OSCConfig and FR3OSCConfig Cartesian ones.
using ControllerConfig = std::variant<JointPDConfig, OSCConfig,
                                      FR3JointControllerConfig, FR3OSCConfig>;

// Whether the config gives a Cartesian controller (else a joint controller).
inline bool is_cartesian(const ControllerConfig& config) {
  return std::holds_alternative<OSCConfig>(config) ||
         std::holds_alternative<FR3OSCConfig>(config);
}

// The controller built from a config, exactly one of the two is set.
struct ControllerPair {
  std::shared_ptr<JointController> joint;
  std::shared_ptr<CartesianController> cartesian;
};

// Builds the controller with a linear interpolator that spreads each goal
// over interpolation_time (the control period of the policy). The joint
// limits are the robot's, as in RobotConfig::joint_limits. With a safety
// config (RobotConfig::safety), the controller's torques go through a
// SafetyStage; timestep is the period of the control loop.
ControllerPair make_controller(const ControllerConfig& config,
                               double interpolation_time,
                               const JointLimits& joint_limits,
                               const std::optional<SafetyConfig>& safety,
                               double timestep);

}  // namespace common
}  // namespace rcs
#endif  // RCS_CONTROLLER_CONFIG_H
