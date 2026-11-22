#ifndef RCS_CONTROLLERS_H
#define RCS_CONTROLLERS_H

#include <Eigen/Core>
#include <memory>
#include <optional>

#include "Controller.h"
#include "Interpolator.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Joint-space PD around the interpolated setpoint:
//   tau = kp (q_d - q) + kd (dq_d - dq) + coriolis
class JointPD : public JointController {
 public:
  JointPD(const VectorXd& kp, const VectorXd& kd,
          std::shared_ptr<JointInterpolator> interpolator);
  void reset(const ControllerState& state) override;
  void set_goal(const VectorXd& q_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  VectorXd kp;
  VectorXd kd;
  std::shared_ptr<JointInterpolator> interpolator;
};

struct OSCConfig {
  Eigen::Vector3d kp_position = Eigen::Vector3d::Constant(150.0);
  Eigen::Vector3d kp_orientation = Eigen::Vector3d::Constant(250.0);
  // Posture task in the null space of the TCP task. nullspace_q empty: hold
  // the configuration of the last reset().
  double kp_nullspace = 10.0;
  VectorXd nullspace_q;
  // Added to the diagonal of the mass matrix (kg m^2 per joint), empty: none.
  // The rigid-body model has no motor inertia, so the wrist joints look almost
  // weightless (FR3 joint 7: 0.0007 kg m^2) and the OSC commands too little
  // torque there to overcome friction: a rotation about the tool axis is not
  // realized. The motor inertias keep the joints in proportion (the coupled
  // OSC moves the joints with the least inertia): for the FR3 the armature of
  // its MJCF, 0.195 on joints 1-4 and 0.074 on joints 5-7. (The FR3 OSC adds
  // 0.1 / 0.5 / 0.5 to joints 5-7 only, with which this OSC turns joints 1, 3
  // and 5 for a rotation about the tool axis.)
  VectorXd residual_mass;
  // Damp towards the setpoint velocity of the interpolator (true) or towards
  // zero velocity (false, as the FR3 OSC). With streamed teleop goals the
  // setpoint velocity jumps with every goal (hand jitter over one period), so
  // the feed-forward kicks the torques at the goal rate.
  bool velocity_feedforward = true;
};

// Operational-space control (Khatib), critically damped (kd = 2 sqrt(kp)),
// with a posture task in the null space:
//   F    = kp e + kd (v_d - v)          e: position and rotation error
//   tau  = J' L F + N' tau_posture + coriolis
// with the task-space inertia L = (J M^-1 J')^-1 and N the dynamically
// consistent null-space projector. Position and orientation are coupled
// through L, so the TCP follows the setpoint as a decoupled second-order
// system when the model is exact.
class OSC : public CartesianController {
 public:
  OSC(const OSCConfig& config,
      std::shared_ptr<CartesianInterpolator> interpolator);
  void reset(const ControllerState& state) override;
  void set_goal(const Pose& pose_goal,
                std::optional<double> duration = std::nullopt) override;
  VectorXd step(const ControllerState& state) override;

 private:
  OSCConfig config;
  VectorXd nullspace_q;
  std::shared_ptr<CartesianInterpolator> interpolator;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_CONTROLLERS_H
