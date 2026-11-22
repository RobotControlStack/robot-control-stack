#ifndef RCS_END_EFFECTOR_H
#define RCS_END_EFFECTOR_H

#include <Eigen/Core>
#include <optional>

#include "Pose.h"

namespace rcs {
namespace common {

// Load fixed to the flange, e.g. a gripper, expressed in the flange frame.
// Same meaning as libfranka's m_total, F_x_Ctotal and I_total.
struct Payload {
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // about the CoM
};

// The tool after the flange (the attachment site): where its TCP is and what
// it weighs. On a Franka, Desk's end effector settings hold the same two
// things (F_T_EE and the mass properties); the backend warns when they differ.
struct EndEffectorConfig {
  // TCP relative to the flange, used by Cartesian goals, observations, IK
  // and the dynamics
  Pose tcp_offset = Pose::Identity();
  // Mass properties for the dynamics of torque control. None: sim uses the
  // MJCF bodies after the flange, hardware what the robot reports (Desk).
  // Set: sim uses it for the controller model and the gravity compensation
  // (as the firmware does with Desk's values), hardware compares it to Desk.
  std::optional<Payload> payload = std::nullopt;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_END_EFFECTOR_H
