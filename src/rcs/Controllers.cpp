#include "rcs/Controllers.h"

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace rcs {
namespace common {

JointPD::JointPD(const VectorXd& kp, const VectorXd& kd,
                 std::shared_ptr<JointInterpolator> interpolator)
    : kp(kp), kd(kd), interpolator(std::move(interpolator)) {
  if (kp.size() != kd.size()) {
    throw std::invalid_argument("kp and kd must have the same size");
  }
  if (!this->interpolator) {
    throw std::invalid_argument("interpolator must not be null");
  }
}

void JointPD::reset(const ControllerState& state) {
  this->interpolator->reset(state.q);
}

void JointPD::set_goal(const VectorXd& q_goal, std::optional<double> duration) {
  this->interpolator->set_goal(q_goal, duration);
}

VectorXd JointPD::step(const ControllerState& state) {
  if (state.q.size() != this->kp.size()) {
    throw std::invalid_argument("state has the wrong number of joints");
  }
  JointSetpoint setpoint = this->interpolator->step(state.dt);
  return this->kp.cwiseProduct(setpoint.q - state.q) +
         this->kd.cwiseProduct(setpoint.dq - state.dq) + state.coriolis;
}

OSC::OSC(const OSCConfig& config,
         std::shared_ptr<CartesianInterpolator> interpolator)
    : config(config), interpolator(std::move(interpolator)) {
  if (!this->interpolator) {
    throw std::invalid_argument("interpolator must not be null");
  }
}

void OSC::reset(const ControllerState& state) {
  this->interpolator->reset(state.tcp_pose);
  this->nullspace_q =
      this->config.nullspace_q.size() > 0 ? this->config.nullspace_q : state.q;
}

void OSC::set_goal(const Pose& pose_goal, std::optional<double> duration) {
  this->interpolator->set_goal(pose_goal, duration);
}

VectorXd OSC::step(const ControllerState& state) {
  if (this->nullspace_q.size() != state.q.size()) {
    throw std::logic_error("call reset() before step()");
  }
  CartesianSetpoint setpoint = this->interpolator->step(state.dt);
  const Eigen::Index n = state.q.size();
  const auto Jp = state.jacobian.topRows<3>();
  const auto Jo = state.jacobian.bottomRows<3>();
  const Eigen::MatrixXd M_inv =
      state.mass.ldlt().solve(Eigen::MatrixXd::Identity(n, n));

  // desired task-space accelerations
  Vector6d velocity = state.jacobian * state.dq;
  Eigen::Vector3d kd_position = 2.0 * this->config.kp_position.cwiseSqrt();
  Eigen::Vector3d kd_orientation =
      2.0 * this->config.kp_orientation.cwiseSqrt();
  Eigen::Vector3d position_error =
      setpoint.pose.translation() - state.tcp_pose.translation();
  Eigen::Vector3d rotation_error =
      rotation_between(state.tcp_pose.quaternion(), setpoint.pose.quaternion());
  Eigen::Vector3d F_position =
      this->config.kp_position.cwiseProduct(position_error) +
      kd_position.cwiseProduct(setpoint.twist.head<3>() - velocity.head<3>());
  Eigen::Vector3d F_orientation =
      this->config.kp_orientation.cwiseProduct(rotation_error) +
      kd_orientation.cwiseProduct(setpoint.twist.tail<3>() -
                                  velocity.tail<3>());

  // decoupled task-space inertias
  Eigen::Matrix3d L_position = (Jp * M_inv * Jp.transpose())
                                   .completeOrthogonalDecomposition()
                                   .pseudoInverse();
  Eigen::Matrix3d L_orientation = (Jo * M_inv * Jo.transpose())
                                      .completeOrthogonalDecomposition()
                                      .pseudoInverse();
  VectorXd tau = Jp.transpose() * (L_position * F_position) +
                 Jo.transpose() * (L_orientation * F_orientation);

  // posture task, projected into the null space of the TCP task
  Eigen::Matrix<double, 6, 6> L =
      (state.jacobian * M_inv * state.jacobian.transpose())
          .completeOrthogonalDecomposition()
          .pseudoInverse();
  Eigen::MatrixXd J_bar = M_inv * state.jacobian.transpose() * L;
  Eigen::MatrixXd N_transposed = Eigen::MatrixXd::Identity(n, n) -
                                 state.jacobian.transpose() * J_bar.transpose();
  double kd_nullspace = 2.0 * std::sqrt(this->config.kp_nullspace);
  VectorXd tau_posture =
      state.mass * (this->config.kp_nullspace * (this->nullspace_q - state.q) -
                    kd_nullspace * state.dq);
  tau += N_transposed * tau_posture;

  return tau + state.coriolis;
}

}  // namespace common
}  // namespace rcs
