#ifndef RCS_LINEAR_INTERPOLATOR_H
#define RCS_LINEAR_INTERPOLATOR_H

#include <optional>

#include "Interpolator.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Moves the setpoint on a straight line from the current setpoint to the goal
// with constant velocity, over `duration` seconds, then holds the goal.
class LinearJointInterpolator : public JointInterpolator {
 public:
  // duration: seconds per segment, typically 1 / policy_rate.
  explicit LinearJointInterpolator(double duration);
  void reset(const VectorXd& q) override;
  void set_goal(const VectorXd& q_goal,
                std::optional<double> duration = std::nullopt) override;
  JointSetpoint step(double dt) override;

 private:
  double default_duration;
  double duration = 0.0;  // of the current segment
  double elapsed = 0.0;   // since the start of the current segment
  VectorXd start;
  VectorXd goal;
  VectorXd q;  // current setpoint
};

// Like LinearJointInterpolator for a TCP pose: straight line for the
// position, slerp for the orientation.
class LinearCartesianInterpolator : public CartesianInterpolator {
 public:
  explicit LinearCartesianInterpolator(double duration);
  void reset(const Pose& pose) override;
  void set_goal(const Pose& pose_goal,
                std::optional<double> duration = std::nullopt) override;
  CartesianSetpoint step(double dt) override;

 private:
  double default_duration;
  double duration = 0.0;
  double elapsed = 0.0;
  bool initialized = false;
  Pose start;
  Pose goal;
  Pose pose;  // current setpoint
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_LINEAR_INTERPOLATOR_H
