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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rcs {
namespace common {

RL::RL(const std::string& urdf_path, size_t max_duration_ms) : rl_data() {
  this->rl_data.mdl = rl::mdl::UrdfFactory().create(urdf_path);
  this->rl_data.kin =
      std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rl_data.mdl);
  this->rl_data.ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(
      this->rl_data.kin.get());
  this->rl_data.ik->setRandomRestarts(this->random_restarts);
  this->rl_data.ik->setEpsilon(this->eps);
  this->rl_data.ik->setDuration(std::chrono::milliseconds(max_duration_ms));
}

std::optional<VectorXd> RL::ik(const Pose& pose, const VectorXd& q0,
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
Pose RL::forward(const VectorXd& q0, const Pose& tcp_offset) {
  // pose is assumed to be in the robots coordinate frame
  this->rl_data.kin->setPosition(q0);
  this->rl_data.kin->forwardPosition();
  rcs::common::Pose pose = this->rl_data.kin->getOperationalPosition(0);
  // apply the tcp offset
  return pose * tcp_offset.inverse();
}

Pin::Pin(const std::string& urdf_path, const std::string& frame_id) : model() {
  pinocchio::urdf::buildModel(urdf_path, this->model);
  this->data = pinocchio::Data(this->model);
  this->FRAME_ID = model.getFrameId(frame_id);
  if (FRAME_ID == -1) {
    throw std::runtime_error(
        frame_id + " frame id could not be found in the provided URDF");
  }
}

std::optional<VectorXd> Pin::ik(const Pose& pose, const VectorXd& q0,
                                const Pose& tcp_offset) {
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();
  VectorXd q(q0);
  const pinocchio::SE3 oMdes(new_pose.rotation_m(), new_pose.translation());
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  bool success = false;
  Vector6d err;
  Eigen::VectorXd v(model.nv);
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3 iMd = data.oMf[this->FRAME_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();
    if (err.norm() < this->eps) {
      success = true;
      break;
    }
    if (i >= this->IT_MAX) {
      success = false;
      break;
    }
    pinocchio::computeFrameJacobian(model, data, q, this->FRAME_ID, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += this->damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * this->DT);
  }
  if (success) {
    return q;
  } else {
    return std::nullopt;
  }
}

Pose Pin::forward(const VectorXd& q0, const Pose& tcp_offset) {
  // pose is assumed to be in the robots coordinate frame
  pinocchio::forwardKinematics(model, data, q0);
  pinocchio::updateFramePlacements(model, data);
  rcs::common::Pose pose(data.oMf[this->FRAME_ID].rotation(),
                         data.oMf[this->FRAME_ID].translation());

  // apply the tcp offset
  return pose * tcp_offset.inverse();
}

}  // namespace common
}  // namespace rcs