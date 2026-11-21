#include "rcs/FR3Controllers.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rcs {
namespace common {

namespace {

// franka::limitRate with franka::kMaxTorqueRate: at most (1000 - 1e-3) Nm/s
// per joint, for the fixed 1 ms cycle libfranka assumes.
constexpr double kMaxTorqueStep = (1000.0 - 1e-3) * 1e-3;

// PInverse from Franka.cpp: SVD pseudo-inverse, singular values below
// epsilon are dropped.
Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& M,
                               double epsilon = 0.00025) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(M.cols(), M.rows());
  for (Eigen::Index i = 0; i < svd.singularValues().size(); i++) {
    double s = svd.singularValues()(i);
    S_inv(i, i) = s < epsilon ? 0.0 : 1.0 / s;
  }
  return svd.matrixV() * S_inv * svd.matrixU().transpose();
}

// The end of both control loops in Franka.cpp: no torque that pushes a joint
// that is closer than 0.1 rad to its limit further, the torque rate limit,
// and the per-joint torque clip.
Vector7d safety_stage(Vector7d tau, const Vector7d& q,
                      const FR3TorqueLimits& limits, Vector7d& last_tau) {
  for (int i = 0; i < 7; i++) {
    if (limits.joint_max[i] - q[i] < 0.1 && tau[i] > 0.) tau[i] = 0.;
    if (q[i] - limits.joint_min[i] < 0.1 && tau[i] < 0.) tau[i] = 0.;
  }
  for (int i = 0; i < 7; i++) {
    tau[i] = last_tau[i] +
             std::clamp(tau[i] - last_tau[i], -kMaxTorqueStep, kMaxTorqueStep);
    tau[i] =
        std::clamp(tau[i], -limits.torque_limit[i], limits.torque_limit[i]);
  }
  last_tau = tau;
  return tau;
}

void check_seven_joints(const ControllerState& state) {
  if (state.q.size() != 7) {
    throw std::invalid_argument("FR3 controllers need 7 joints");
  }
}

}  // namespace

FR3JointController::FR3JointController(
    const FR3JointControllerConfig& config,
    std::shared_ptr<JointInterpolator> interpolator)
    : config(config), interpolator(std::move(interpolator)) {
  if (!this->interpolator) {
    throw std::invalid_argument("interpolator must not be null");
  }
}

void FR3JointController::reset(const ControllerState& state) {
  check_seven_joints(state);
  this->interpolator->reset(state.q);
  this->last_tau.setZero();
}

void FR3JointController::set_goal(const VectorXd& q_goal,
                                  std::optional<double> duration) {
  this->interpolator->set_goal(q_goal, duration);
}

VectorXd FR3JointController::step(const ControllerState& state) {
  check_seven_joints(state);
  const Vector7d q = state.q;
  const Vector7d dq = state.dq;
  Vector7d desired_q = this->interpolator->step(state.dt).q;

  Vector7d tau = this->config.kp.cwiseProduct(desired_q - q) -
                 this->config.kd.cwiseProduct(dq);
  tau += state.coriolis;
  return safety_stage(tau, q, this->config.limits, this->last_tau);
}

FR3OSC::FR3OSC(const FR3OSCConfig& config,
               std::shared_ptr<CartesianInterpolator> interpolator)
    : config(config), interpolator(std::move(interpolator)) {
  if (!this->interpolator) {
    throw std::invalid_argument("interpolator must not be null");
  }
}

void FR3OSC::reset(const ControllerState& state) {
  check_seven_joints(state);
  this->interpolator->reset(state.tcp_pose);
  this->last_tau.setZero();
}

void FR3OSC::set_goal(const Pose& pose_goal, std::optional<double> duration) {
  this->interpolator->set_goal(pose_goal, duration);
}

VectorXd FR3OSC::step(const ControllerState& state) {
  check_seven_joints(state);
  const FR3OSCConfig& cfg = this->config;
  const Vector7d q = state.q;
  const Vector7d dq = state.dq;
  CartesianSetpoint setpoint = this->interpolator->step(state.dt);

  Eigen::Matrix3d Kp_p = cfg.kp_position.asDiagonal();
  Eigen::Matrix3d Kp_r = Eigen::Matrix3d::Identity() * cfg.kp_orientation;
  Eigen::Matrix3d Kd_p = 2.0 * Kp_p.cwiseSqrt();
  Eigen::Matrix3d Kd_r = 2.0 * Kp_r.cwiseSqrt();

  Eigen::Matrix<double, 7, 7> M = state.mass;
  M += cfg.residual_mass.asDiagonal();
  const Eigen::Matrix<double, 6, 7> jacobian = state.jacobian;
  const Eigen::Matrix<double, 3, 7> jacobian_pos = jacobian.topRows<3>();
  const Eigen::Matrix<double, 3, 7> jacobian_ori = jacobian.bottomRows<3>();

  // errors of the TCP, in the base frame
  Eigen::Vector3d pos_EE = state.tcp_pose.translation();
  Eigen::Quaterniond quat_EE = state.tcp_pose.quaternion();
  Eigen::Quaterniond desired_quat = setpoint.pose.quaternion();
  if (desired_quat.coeffs().dot(quat_EE.coeffs()) < 0.0) {
    quat_EE.coeffs() << -quat_EE.coeffs();
  }
  Eigen::Vector3d pos_error = setpoint.pose.translation() - pos_EE;
  Eigen::Quaterniond quat_error(desired_quat.inverse() * quat_EE);
  Eigen::Vector3d ori_error(quat_error.x(), quat_error.y(), quat_error.z());
  ori_error = -state.tcp_pose.rotation_m() * ori_error;

  // task-space inertias and the null-space projector
  Eigen::Matrix<double, 7, 7> M_inv = M.inverse();
  Eigen::MatrixXd Lambda =
      pseudo_inverse(jacobian * M_inv * jacobian.transpose());
  Eigen::Matrix<double, 7, 6> J_inv = M_inv * jacobian.transpose() * Lambda;
  Eigen::Matrix<double, 7, 7> Nullspace =
      Eigen::Matrix<double, 7, 7>::Identity() -
      jacobian.transpose() * J_inv.transpose();
  Eigen::MatrixXd Lambda_pos =
      pseudo_inverse(jacobian_pos * M_inv * jacobian_pos.transpose());
  Eigen::MatrixXd Lambda_ori =
      pseudo_inverse(jacobian_ori * M_inv * jacobian_ori.transpose());

  pos_error = pos_error.unaryExpr(
      [](double x) { return (std::abs(x) < 1e-4) ? 0. : x; });
  ori_error = ori_error.unaryExpr(
      [](double x) { return (std::abs(x) < 5e-3) ? 0. : x; });

  Vector7d tau =
      jacobian_pos.transpose() *
          (Lambda_pos * (Kp_p * pos_error - Kd_p * (jacobian_pos * dq))) +
      jacobian_ori.transpose() *
          (Lambda_ori * (Kp_r * ori_error - Kd_r * (jacobian_ori * dq)));

  // posture in the null space
  tau += Nullspace * (cfg.nullspace_q - q);

  // joint-limit avoidance in the null space
  Vector7d avoidance = Vector7d::Zero();
  for (int i = 0; i < 7; i++) {
    double to_max = cfg.limits.joint_max[i] - q[i];
    double to_min = q[i] - cfg.limits.joint_min[i];
    if (to_max < 0.25 && to_max > 0.1)
      avoidance[i] += -cfg.avoidance_weights[i] * to_max;
    if (to_min < 0.25 && to_min > 0.1)
      avoidance[i] += cfg.avoidance_weights[i] * to_min;
  }
  tau += Nullspace * avoidance;

  tau += state.coriolis;
  return safety_stage(tau, q, cfg.limits, this->last_tau);
}

}  // namespace common
}  // namespace rcs
