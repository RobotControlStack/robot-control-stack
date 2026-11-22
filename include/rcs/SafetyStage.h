#ifndef RCS_SAFETY_STAGE_H
#define RCS_SAFETY_STAGE_H

#include <limits>
#include <memory>
#include <optional>

#include "Controller.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Limits applied to the torques of any controller, a property of the robot
// (RobotConfig::safety, set from rcs.ROBOTS in Python).
struct SafetyConfig {
  // per-joint torque clip in Nm, empty: no clip
  VectorXd torque_limit;
  // per-joint torque rate limit in Nm/s (franka::limitRate on the FR3)
  double max_torque_rate = std::numeric_limits<double>::infinity();
  // no torque that pushes a joint closer than this to its limit (rad)
  double joint_limit_margin = 0.1;
};

// The safety stage of Franka.cpp, for any robot and controller. In this order:
// 1. joint-limit guard: a joint closer than joint_limit_margin to a limit gets
//    no torque towards it,
// 2. rate limit: each joint torque changes by at most
//    max_torque_rate * timestep from the previous output per step (starting
//    from zero after reset). timestep is the nominal period of the control
//    loop, not the measured one: libfranka limits per command, assuming 1 ms.
// 3. clip to torque_limit.
class SafetyStage {
 public:
  SafetyStage(const SafetyConfig& config, const JointLimits& joint_limits,
              double timestep);
  // the output starts from zero torque again
  void reset(const ControllerState& state);
  VectorXd apply(const VectorXd& tau, const ControllerState& state);

 private:
  SafetyConfig config;
  JointLimits joint_limits;
  double timestep;
  VectorXd last_tau;
};

// A controller whose torques go through a SafetyStage.
class SafeJointController : public JointController {
 public:
  SafeJointController(std::shared_ptr<JointController> controller,
                      const SafetyConfig& config,
                      const JointLimits& joint_limits, double timestep);
  void reset(const ControllerState& state) override;
  void set_goal(const VectorXd& q_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  std::shared_ptr<JointController> controller;
  SafetyStage stage;
};

class SafeCartesianController : public CartesianController {
 public:
  SafeCartesianController(std::shared_ptr<CartesianController> controller,
                          const SafetyConfig& config,
                          const JointLimits& joint_limits, double timestep);
  void reset(const ControllerState& state) override;
  void set_goal(const Pose& pose_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  std::shared_ptr<CartesianController> controller;
  SafetyStage stage;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_SAFETY_STAGE_H
