#include "IK.h"

#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Exception.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/UrdfFactory.h>

namespace rcs {
namespace common {

IK::IK(const std::string& urdf_path, size_t max_duration_ms) : rl() {
  this->rl = RL();

  this->rl.mdl = rl::mdl::UrdfFactory().create(urdf_path);
  this->rl.kin = std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rl.mdl);
  this->rl.ik =
      std::make_shared<rl::mdl::JacobianInverseKinematics>(this->rl.kin.get());
  this->rl.ik->setRandomRestarts(0);
  this->rl.ik->setEpsilon(1e-3);
  this->rl.ik->setDuration(std::chrono::milliseconds(max_duration_ms));
}

std::optional<Vector7d> IK::ik(const Pose& pose, const Vector7d& q0,
                               const Pose& tcp_offset) {
  // pose is assumed to be in the robots coordinate frame
  this->rl.kin->setPosition(q0);
  this->rl.kin->forwardPosition();
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();

  this->rl.ik->addGoal(new_pose.affine_matrix(), 0);
  bool success = this->rl.ik->solve();
  if (success) {
    // is this forward needed and is it mabye possible to call
    // this on the model?
    this->rl.kin->forwardPosition();
    return this->rl.kin->getPosition();
  } else {
    return std::nullopt;
  }
}

}  // namespace common
}  // namespace rcs