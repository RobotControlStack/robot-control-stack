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

RL::RL(const std::string& urdf_path, size_t max_duration_ms) : rl_data() {
  this->rl_data.mdl = rl::mdl::UrdfFactory().create(urdf_path);
  this->rl_data.kin =
      std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rl_data.mdl);
  this->rl_data.ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(
      this->rl_data.kin.get());
  this->rl_data.ik->setRandomRestarts(0);
  this->rl_data.ik->setEpsilon(1e-3);
  this->rl_data.ik->setDuration(std::chrono::milliseconds(max_duration_ms));
}

std::optional<Vector7d> RL::ik(const Pose& pose, const Vector7d& q0,
                               const Pose& tcp_offset) {
  // pose is assumed to be in the robots coordinate frame
  this->rl_data.kin->setPosition(q0);
  this->rl_data.kin->forwardPosition();
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();

  this->rl_data.ik->addGoal(new_pose.affine_matrix(), 0);
  bool success = this->rl_data.ik->solve();
  if (success) {
    // is this forward needed and is it mabye possible to call
    // this on the model?
    this->rl_data.kin->forwardPosition();
    return this->rl_data.kin->getPosition();
  } else {
    return std::nullopt;
  }
}

}  // namespace common
}  // namespace rcs