#include "Pose.h"

namespace rcs {
namespace common {

Eigen::Vector3d IdentityTranslation() { return Eigen::Vector3d::Zero(); }
Eigen::Matrix3d IdentityRotMatrix() { return Eigen::Matrix3d::Identity(); }
Eigen::Quaterniond IdentityRotQuart() { return Eigen::Quaterniond::Identity(); }
Eigen::Vector4d IdentityRotQuartVec() { return IdentityRotQuart().coeffs(); }

Eigen::Matrix4d FrankaHandTCPOffset() {
  return (Eigen::Matrix4d() << 0.707, 0.707, 0, 0, -0.707, 0.707, 0, 0, 0, 0, 1,
          0.1034, 0, 0, 0, 1)
      .finished();
}

// CONSTRUCTORS

Pose::Pose() {
  this->m_translation = IdentityTranslation();
  this->m_rotation = IdentityRotQuart();
}

Pose::Pose(const Eigen::Affine3d &pose_matrix) {
  this->m_translation = pose_matrix.translation();
  this->m_rotation = Eigen::Quaterniond(pose_matrix.rotation());
  this->m_rotation.normalize();
}

Pose::Pose(const std::array<double, 16> &pose)
    : Pose(array2eigen<4, 4>(pose)) {}

Pose::Pose(const Eigen::Matrix4d &pose) {
  Eigen::Affine3d affine_pose = Eigen::Affine3d(pose);
  this->m_translation = affine_pose.translation();
  this->m_rotation = Eigen::Quaterniond(affine_pose.rotation());
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Matrix3d &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = Eigen::Quaterniond(rotation);
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Vector4d &quaternion,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = Eigen::Quaterniond(quaternion);
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Quaterniond &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = rotation;
  this->m_rotation.normalize();
}

Pose::Pose(const RPY &rotation, const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = rotation.as_quaternion();
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Vector3d &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = RPY(rotation).as_quaternion();
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = IdentityRotQuart();
}

Pose::Pose(const Eigen::Quaterniond &quaternion) {
  this->m_translation = IdentityTranslation();
  this->m_rotation = quaternion;
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Vector4d &quaternion) {
  this->m_translation = IdentityTranslation();
  this->m_rotation = Eigen::Quaterniond(quaternion);
  this->m_rotation.normalize();
}

Pose::Pose(const RPY &rpy) {
  this->m_translation = IdentityTranslation();
  this->m_rotation = rpy.as_quaternion();
  this->m_rotation.normalize();
}

Pose::Pose(const Eigen::Matrix3d &rotation) {
  this->m_translation = IdentityTranslation();
  this->m_rotation = Eigen::Quaterniond(rotation);
}

Pose::Pose(const Pose &pose) {
  this->m_translation = pose.translation();
  this->m_rotation = pose.quaternion();
}

// GETTERS

Eigen::Vector3d Pose::translation() const { return this->m_translation; }

Eigen::Matrix3d Pose::rotation_m() const {
  return this->m_rotation.toRotationMatrix();
}

Eigen::Vector4d Pose::rotation_q() const { return this->m_rotation.coeffs(); }

Eigen::Quaterniond Pose::quaternion() const { return this->m_rotation; }

Eigen::Affine3d Pose::affine_matrix() const {
  Eigen::Affine3d pose =
      Eigen::Translation3d(this->m_translation) * this->m_rotation;
  return pose;
}

Eigen::Matrix4d Pose::pose_matrix() const {
  return this->affine_matrix().matrix();
}

std::array<double, 16> Pose::affine_array() const {
  return eigen2array<4, 4>(this->affine_matrix().matrix());
}

RPY Pose::rotation_rpy() const {
  // ZYX, roll, pitch, yaw
  Eigen::Vector3d rpy_vec =
      this->m_rotation.toRotationMatrix().eulerAngles(2, 1, 0);
  return RPY{rpy_vec.z(), rpy_vec.y(), rpy_vec.x()};
}

Pose Pose::interpolate(const Pose &dest_pose, double progress) const {
  if (progress > 1) {
    progress = 1;
  }
  Eigen::Vector3d pos_result =
      this->translation() +
      (dest_pose.translation() - translation()) * progress;
  Eigen::Quaterniond quat_start, quat_end, quat_result;
  quat_start = this->quaternion();
  quat_end = dest_pose.quaternion();
  quat_result = quat_start.slerp(progress, quat_end);
  Pose result_pose(quat_result, pos_result);
  return result_pose;
}

Vector6d Pose::xyzrpy() const {
  Vector6d xyzrpy;
  xyzrpy.head(3) = this->translation();
  xyzrpy.tail(3) = this->rotation_rpy().as_vector();
  return xyzrpy;
}

std::string Pose::str() const {
  std::stringstream ss;

  RPY angles = this->rotation_rpy();

  ss << this->affine_matrix().matrix() << std::endl;
  ss << "roll: " << angles.roll << "\tpitch: " << angles.pitch
     << "\tyaw: " << angles.yaw;
  return ss.str();
}

Pose Pose::operator*(const Pose &pose_b) const {
  Eigen::Vector3d trans =
      this->m_rotation * pose_b.translation() + this->m_translation;
  Eigen::Quaterniond rot = this->m_rotation * pose_b.m_rotation;
  return Pose(rot, trans);
}

double Pose::total_angle() const {
  return this->m_rotation.angularDistance(IdentityRotQuart());
}

Pose Pose::set_angle(double angle) const {
  Eigen::Quaterniond rot = this->m_rotation;
  rot.coeffs()[3] = cos(angle / 2);
  return Pose(rot, this->m_translation);
}

Pose Pose::inverse() const {
  auto new_rot = this->m_rotation.conjugate();
  return Pose(new_rot, -(new_rot * this->m_translation));
}

bool Pose::is_close(const Pose &other, double eps_r, double eps_t) const {
  return (this->translation() - other.translation()).lpNorm<1>() < eps_t &&
         this->quaternion().angularDistance(other.quaternion()) < eps_r;
}

}  // namespace common
}  // namespace rcs
