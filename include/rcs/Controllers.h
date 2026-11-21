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
