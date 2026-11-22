#ifndef RCS_FR3_CONTROLLERS_H
#define RCS_FR3_CONTROLLERS_H

#include <Eigen/Core>
#include <memory>
#include <optional>

#include "Controller.h"
#include "Interpolator.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// The torque controllers that run on the FR3, ported one to one from
// Franka::joint_controller() and Franka::osc() (deoxys), so that the sim can
// run them too. Differences to the original:
// - The torque rate limit uses the controller's own previous output instead
//   of franka::RobotState::tau_J_d (on the robot they are the same).
// - Dynamics come from the ControllerState (PinocchioDynamics) instead of
//   franka::Model, which computes the same quantities.

// - The safety stage at the end of both loops (joint-limit guard, torque rate
//   limit, torque clip) is not part of the controllers, it is the SafetyStage
//   that RobotConfig::safety adds to any controller (see SafetyStage.h).

struct FR3JointControllerConfig {
  Vector7d kp =
      (Vector7d() << 100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0).finished();
  Vector7d kd =
      (Vector7d() << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0).finished();
};

// Joint impedance: tau = kp (q_d - q) - kd dq + coriolis. Note: no setpoint
// velocity feed-forward, as on the robot.
class FR3JointController : public JointController {
 public:
  FR3JointController(const FR3JointControllerConfig& config,
                     std::shared_ptr<JointInterpolator> interpolator);
  void reset(const ControllerState& state) override;
  void set_goal(const VectorXd& q_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  FR3JointControllerConfig config;
  std::shared_ptr<JointInterpolator> interpolator;
};

struct FR3OSCConfig {
  Eigen::Vector3d kp_position = Eigen::Vector3d::Constant(150.0);  // kp_p
  double kp_orientation = 250.0;                                   // kp_r
  // posture held in the null space (unit gain)
  Vector7d nullspace_q =
      (Vector7d() << 0.09017809387254755, -0.9824203501652151,
       0.030509718397568178, -2.694229634937343, 0.057700675144720104,
       1.860298714876101, 0.8713759453244422)
          .finished();
  // added to the diagonal of the mass matrix
  Vector7d residual_mass =
      (Vector7d() << 0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5).finished();
  // null-space push away from joints that are 0.1 to 0.25 rad from a limit
  Vector7d avoidance_weights =
      (Vector7d() << 1.0, 1.0, 1.0, 1.0, 1.0, 10.0, 10.0).finished();
};

// Decoupled operational-space control as on the FR3 (deoxys): separate
// position and orientation task inertias, damping on the TCP velocity only,
// orientation error from the quaternion vector part, small errors cut off,
// posture and joint-limit avoidance in the null space. The joint limits are
// the robot's (RobotConfig::joint_limits), for the avoidance.
class FR3OSC : public CartesianController {
 public:
  FR3OSC(const FR3OSCConfig& config,
         std::shared_ptr<CartesianInterpolator> interpolator,
         const JointLimits& joint_limits);
  void reset(const ControllerState& state) override;
  void set_goal(const Pose& pose_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  FR3OSCConfig config;
  std::shared_ptr<CartesianInterpolator> interpolator;
  Vector7d joint_min, joint_max;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_FR3_CONTROLLERS_H
