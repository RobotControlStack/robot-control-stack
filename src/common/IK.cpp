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

#include "utils.h"

namespace rcs {
namespace common {
// RelaxedIK* ik = relaxed_ik_new("./fr3.yaml");
// double target_pos[3] = {0.3, 0.2, 0.2};
// double target_quat[4] = {1, 0, 0, 0};
// double tolerances[6] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01 };
// auto ret = solve(ik, target_pos, 3, target_quat, 4, tolerances, 6);
// std::cout << ret.length << std::endl;
// for (size_t i = 0; i < ret.length; ++i) {
//   std::cout << ret.data[i] << " ";
// }
// std::cout << std::endl;
// relaxed_ik_free(ik);

IK::IK(const std::string& urdf_path, size_t max_duration_ms) : rl() {
  this->rl = RL();
  this->relaxed_ik = relaxed_ik_new("./fr3.yaml");
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
  double target_pos[3];
  double target_quat[4];
  target_pos[0] = new_pose.translation()[0];
  target_pos[1] = new_pose.translation()[1];
  target_pos[2] = new_pose.translation()[2];
  target_quat[0] = new_pose.quaternion().x();
  target_quat[1] = new_pose.quaternion().y();
  target_quat[2] = new_pose.quaternion().z();
  target_quat[3] = new_pose.quaternion().w();
  double tolerances[6] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01};
  auto ret =
      solve(this->relaxed_ik, target_pos, 3, target_quat, 4, tolerances, 6);
  Vector7d angles(ret.data);
  // TODO: we need to save previous cartesian poses so we have a trajectory,
  // then we can add onto that trajectory by appending the next target pose.
  // That way the relaxed ik solver can work with an entire trajectory instead
  // of individual steps
  return angles;
  // double target_pos[3] = {0.3, 0.2, 0.2};
  // double target_quat[4] = {1, 0, 0, 0};
  // double tolerances[6] = {0.1, 0.1, 0.1, 0.01, 0.01, 0.01 };
  // auto ret = solve(ik, target_pos, 3, target_quat, 4, tolerances, 6);

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
