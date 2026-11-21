#ifndef RCS_INTERPOLATOR_H
#define RCS_INTERPOLATOR_H

#include <optional>

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
//
// Timing: every goal starts a segment that takes a fixed duration, set by
// the implementation (typically one policy period, 1 / policy_rate). The
// interpolator counts time only through the dt passed to step(), so it runs
// the same in sim and on hardware. Once the segment is finished, the setpoint
// holds the goal with zero velocity until the next goal arrives.
//
// A new segment starts at the current setpoint, never at the previous goal or
// the measured robot state, so the setpoint doesn't jump when a goal arrives
// before the previous segment is finished. The measured state is only used
// through reset().
class JointInterpolator {
 public:
  virtual ~JointInterpolator() = default;
  // Hold q, with zero velocity.
  virtual void reset(const VectorXd& q) = 0;
  // Start moving from the current setpoint towards q_goal. duration
  // overrides the segment duration for this goal only, e.g. for a slow
  // approach when control starts far from the goal.
  virtual void set_goal(const VectorXd& q_goal,
                        std::optional<double> duration = std::nullopt) = 0;
  // Advance by dt seconds and return the new setpoint.
  virtual JointSetpoint step(double dt) = 0;
};

class CartesianInterpolator {
 public:
  virtual ~CartesianInterpolator() = default;
  virtual void reset(const Pose& pose) = 0;
  virtual void set_goal(const Pose& pose_goal,
                        std::optional<double> duration = std::nullopt) = 0;
  virtual CartesianSetpoint step(double dt) = 0;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_INTERPOLATOR_H
