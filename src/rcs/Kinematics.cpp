#include "rcs/Kinematics.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rcs {
namespace common {

Pin::Pin(const std::string& path, const std::string& frame_id, bool urdf = true)
    : model() {
  if (urdf) {
    pinocchio::urdf::buildModel(path, this->model);
  } else {
    pinocchio::mjcf::buildModel(path, this->model);
  }
  this->data = pinocchio::Data(this->model);
  this->FRAME_ID = model.getFrameId(frame_id);
  if (FRAME_ID == -1) {
    throw std::runtime_error(
        frame_id + " frame id could not be found in the provided URDF");
  }
}

std::optional<VectorXd> Pin::inverse(const Pose& pose, const VectorXd& q0,
                                     const Pose& tcp_offset) {
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();
  VectorXd q(model.nq);
  q.setZero();
  q.head(q0.size()) = q0;
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
  VectorXd q(model.nq);
  q.setZero();
  q.head(q0.size()) = q0;
  pinocchio::framesForwardKinematics(model, data, q);
  rcs::common::Pose pose(data.oMf[this->FRAME_ID].rotation(),
                         data.oMf[this->FRAME_ID].translation());

  // apply the tcp offset
  return pose * tcp_offset.inverse();
}

}  // namespace common
}  // namespace rcs