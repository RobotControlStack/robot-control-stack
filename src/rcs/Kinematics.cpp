#include "rcs/Kinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rcs {
namespace common {

Pin::Pin(const std::string& path, const std::string& frame_id, bool urdf,
         const VectorXd& nullspace_q, double nullspace_gain,
         bool enforce_limits)
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

  this->q_lower = this->model.lowerPositionLimit;
  this->q_upper = this->model.upperPositionLimit;
  const double inf = std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < this->q_lower.size(); i++) {
    if (!std::isfinite(this->q_lower[i]) || !std::isfinite(this->q_upper[i]) ||
        this->q_lower[i] >= this->q_upper[i]) {
      this->q_lower[i] = -inf;
      this->q_upper[i] = inf;
    }
  }
  this->enforce_limits = enforce_limits;

  this->nullspace_gain = nullspace_gain;
  this->nullspace_q = VectorXd::Zero(this->model.nq);
  const Eigen::Index n =
      std::min<Eigen::Index>(nullspace_q.size(), this->nullspace_q.size());
  this->nullspace_q.head(n) = nullspace_q.head(n);
}

std::optional<VectorXd> Pin::inverse(const Pose& pose, const VectorXd& q0,
                                     const Pose& tcp_offset) {
  rcs::common::Pose new_pose = pose * tcp_offset.inverse();
  const pinocchio::SE3 oMdes(new_pose.rotation_m(), new_pose.translation());
  VectorXd q_seed(model.nq);
  q_seed.setZero();
  q_seed.head(q0.size()) = q0;
  // a posture step larger than the remaining distance overshoots
  const double nullspace_gain = std::min(this->nullspace_gain, 1.0 / this->DT);
  if (nullspace_gain > 0.0) {
    VectorXd q = q_seed;
    if (this->solve(oMdes, q, nullspace_gain)) {
      return q;
    }
  }
  if (this->solve(oMdes, q_seed, 0.0)) {
    return q_seed;
  }
  return std::nullopt;
}

bool Pin::solve(const pinocchio::SE3& oMdes, VectorXd& q,
                double nullspace_gain) {
  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();
  Vector6d err;
  Eigen::VectorXd v(model.nv);
  for (int i = 0;; i++) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3 iMd = data.oMf[this->FRAME_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector();
    const bool posture_pending = nullspace_gain > 0.0 && i == 0;
    if (err.norm() < this->eps && !posture_pending) {
      return true;
    }
    if (i >= this->IT_MAX) {
      return false;
    }
    pinocchio::computeFrameJacobian(model, data, q, this->FRAME_ID, J);
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += this->damp;

    if (nullspace_gain > 0.0) {
      Eigen::MatrixXd Jpinv =
          J.transpose() *
          JJt.ldlt().solve(pinocchio::Data::Matrix6::Identity());
      v.noalias() = -Jpinv * err;

      VectorXd dq_ns(model.nv);
      pinocchio::difference(model, q, this->nullspace_q, dq_ns);
      const Eigen::MatrixXd N =
          Eigen::MatrixXd::Identity(model.nv, model.nv) - Jpinv * J;
      v.noalias() += N * (nullspace_gain * dq_ns);
    } else {
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    }

    q = pinocchio::integrate(model, q, v * this->DT);
    if (this->enforce_limits) {
      q = q.cwiseMax(this->q_lower).cwiseMin(this->q_upper);
    }
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
  return pose * tcp_offset;
}

}  // namespace common
}  // namespace rcs