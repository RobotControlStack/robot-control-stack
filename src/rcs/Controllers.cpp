#include "rcs/Controllers.h"

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace rcs {
namespace common {

JointPD::JointPD(const VectorXd& kp, const VectorXd& kd,
                 std::shared_ptr<JointInterpolator> interpolator,
                 bool velocity_feedforward)
    : kp(kp),
      kd(kd),
      interpolator(std::move(interpolator)),
      velocity_feedforward(velocity_feedforward) {
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
  const VectorXd dq_desired =
      this->velocity_feedforward ? setpoint.dq : VectorXd::Zero(state.q.size());
  return this->kp.cwiseProduct(setpoint.q - state.q) +
         this->kd.cwiseProduct(dq_desired - state.dq) + state.coriolis;
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
  const auto& J = state.jacobian;
  Eigen::MatrixXd M = state.mass;
  if (this->config.residual_mass.size() != 0) {
    if (this->config.residual_mass.size() != n) {
      throw std::invalid_argument("residual_mass needs one entry per joint");
    }
    M.diagonal() += this->config.residual_mass;
  }
  const Eigen::MatrixXd M_inv = M.ldlt().solve(Eigen::MatrixXd::Identity(n, n));

  // desired TCP acceleration: critically damped spring to the setpoint
  Vector6d kp;
  kp << this->config.kp_position, this->config.kp_orientation;
  Vector6d kd = 2.0 * kp.cwiseSqrt();
  Vector6d error;
  error << setpoint.pose.translation() - state.tcp_pose.translation(),
      rotation_between(state.tcp_pose.quaternion(), setpoint.pose.quaternion());
  const Vector6d v_desired =
      this->config.velocity_feedforward ? setpoint.twist : Vector6d::Zero();
  Vector6d F =
      kp.cwiseProduct(error) + kd.cwiseProduct(v_desired - J * state.dq);

  // task-space inertia
  Eigen::Matrix<double, 6, 6> L = (J * M_inv * J.transpose())
                                      .completeOrthogonalDecomposition()
                                      .pseudoInverse();
  VectorXd tau = J.transpose() * (L * F);

  // posture task, projected into the dynamically consistent null space
  Eigen::MatrixXd J_bar = M_inv * J.transpose() * L;
  Eigen::MatrixXd N_transposed =
      Eigen::MatrixXd::Identity(n, n) - J.transpose() * J_bar.transpose();
  double kd_nullspace = 2.0 * std::sqrt(this->config.kp_nullspace);
  VectorXd tau_posture =
      M * (this->config.kp_nullspace * (this->nullspace_q - state.q) -
           kd_nullspace * state.dq);
  tau += N_transposed * tau_posture;

  return tau + state.coriolis;
}

}  // namespace common
}  // namespace rcs
