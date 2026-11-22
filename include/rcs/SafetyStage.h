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

  // Joint-limit guard, in joint space, per joint (an empty vector switches
  // that part off). The velocity allowed towards a limit at distance d is
  //   v(d) = min(max_velocity, max(0, sqrt(2 deceleration d) -
  //   velocity_offset))
  // as Franka's position based velocity limits (the robot's URDF has the
  // parameters); velocity_offset empty means 0, the plain sqrt(2 a d). v is
  // zero at d0 = velocity_offset^2 / (2 deceleration), the zero-velocity
  // point in front of the limit.
  VectorXd max_velocity;
  VectorXd deceleration;
  VectorXd velocity_offset;
  // damper: damping (Nm s/rad) on the velocity towards a limit above
  // velocity_margin * v(d); slower motion and motion away from a limit are
  // not affected
  VectorXd limit_damping;
  double velocity_margin = 0.8;
  // spring: torque away from a limit, growing from 0 at spring_range (rad) in
  // front of the zero-velocity point to spring_torque (Nm) at it; over the
  // same zone the controller's torque towards the limit fades out to 0, so
  // that no controller can push a joint past the zero-velocity point
  VectorXd spring_torque;
  double spring_range = 0.05;
};

// Limits on the torques of any robot and controller. In this order:
// 1. joint-limit guard (see SafetyConfig): the damper slows a joint that
//    approaches a limit faster than allowed there; in a small zone in front of
//    the limit the controller's torque towards it fades out and a spring
//    pushes back, like a soft end stop,
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
