#ifndef RCS_CONTROLLER_H
#define RCS_CONTROLLER_H

#include <Eigen/Core>
#include <optional>

#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Everything a torque controller needs to know about one robot. Computed by
// PinocchioDynamics from the measured q and dq on every servo tick.
// Cartesian quantities are in the robot base frame, at the TCP.
struct ControllerState {
  VectorXd q;
  VectorXd dq;
  Eigen::MatrixXd mass;
  VectorXd coriolis;  // C(q, dq) * dq
  VectorXd gravity;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;  // rows: [linear; angular]
  Pose tcp_pose;
  double dt = 0.0;  // seconds since the previous step()
};

// Controllers return joint torques WITHOUT gravity compensation. The backend
// compensates gravity the way the robot does (see the spec in
// docs/development/controller_interface.md).
// The interfaces aren't thread-safe: a backend that calls set_goal() and step()
// from different threads has to serialise the calls.

class JointController {
 public:
  virtual ~JointController() = default;
  // Hold the current configuration.
  virtual void reset(const ControllerState& state) = 0;
  // Called at policy rate. duration overrides the interpolator's segment
  // duration for this goal only (see JointInterpolator::set_goal).
  virtual void set_goal(const VectorXd& q_goal,
                        std::optional<double> duration = std::nullopt) = 0;
  // Called at servo rate. Returns joint torques without gravity.
  virtual VectorXd step(const ControllerState& state) = 0;
};

class CartesianController {
 public:
  virtual ~CartesianController() = default;
  virtual void reset(const ControllerState& state) = 0;
  // TCP goal in the robot base frame.
  virtual void set_goal(const Pose& pose_goal,
                        std::optional<double> duration = std::nullopt) = 0;
  virtual VectorXd step(const ControllerState& state) = 0;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_CONTROLLER_H
