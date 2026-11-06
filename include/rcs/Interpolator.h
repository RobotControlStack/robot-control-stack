#ifndef RCS_INTERPOLATOR_H
#define RCS_INTERPOLATOR_H

#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

// Desired joint state for one servo tick.
struct JointSetpoint {
  VectorXd q;
  VectorXd dq;
};

// Desired TCP state for one servo tick, in the robot base frame.
struct CartesianSetpoint {
  Pose pose;
  Vector6d twist;  // [linear velocity; angular velocity]
};

// Turns sparse goals (policy rate) into a smooth setpoint (servo rate).
class JointInterpolator {
 public:
  virtual ~JointInterpolator() = default;
  // Hold q, with zero velocity.
  virtual void reset(const VectorXd& q) = 0;
  // Start moving from the current setpoint towards q_goal.
  virtual void set_goal(const VectorXd& q_goal) = 0;
  // Advance by dt seconds and return the new setpoint.
  virtual JointSetpoint step(double dt) = 0;
};

class CartesianInterpolator {
 public:
  virtual ~CartesianInterpolator() = default;
  virtual void reset(const Pose& pose) = 0;
  virtual void set_goal(const Pose& pose_goal) = 0;
  virtual CartesianSetpoint step(double dt) = 0;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_INTERPOLATOR_H
