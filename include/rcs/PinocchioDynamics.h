#ifndef RCS_PINOCCHIO_DYNAMICS_H
#define RCS_PINOCCHIO_DYNAMICS_H

#include <Eigen/Core>
#include <string>

#include "Controller.h"
#include "EndEffector.h"
#include "Pose.h"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "utils.h"

namespace rcs {
namespace common {

// Rigid-body dynamics of one arm, used by sim and real alike to fill the
// ControllerState. Computes the same quantities as libfranka's franka::Model,
// which is itself built on Pinocchio.
//
// The model file must contain only the arm, with the robot base at the model
// root. Joint armature (rotor inertia) in the file is ignored, as in
// franka::Model.
class PinocchioDynamics {
 public:
  // model_path: MJCF or URDF file of the arm.
  // flange_frame: frame the payload and the TCP are attached to, e.g. the
  //   attachment site.
  // tcp_offset: TCP relative to the flange.
  PinocchioDynamics(const std::string& model_path, bool urdf,
                    const std::string& flange_frame,
                    const Pose& tcp_offset = Pose::Identity());

  void set_payload(const Payload& payload);
  Payload get_payload() const;
  void set_tcp_offset(const Pose& tcp_offset);
  // Gravity acceleration in the robot base frame, default (0, 0, -9.81).
  void set_gravity(const Eigen::Vector3d& gravity);
  size_t dof() const;

  // All Cartesian quantities are in the robot base frame, at the TCP.
  ControllerState compute(const VectorXd& q, const VectorXd& dq, double dt);

 private:
  pinocchio::Model model;
  pinocchio::Data data;
  pinocchio::JointIndex flange_joint;  // joint that moves the flange
  pinocchio::SE3 flange_placement;     // flange in that joint's frame
  pinocchio::Inertia arm_inertia;      // of that joint's body, no payload
  pinocchio::FrameIndex tcp_frame;
  Payload payload;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_PINOCCHIO_DYNAMICS_H
