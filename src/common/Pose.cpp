#include "Pose.h"

#include "utils.h"

namespace rcs {
namespace common {

Eigen::Vector3d IdentityTranslation() { return Eigen::Vector3d::Zero(); }
Eigen::Matrix3d IdentityRotation() { return Eigen::Matrix3d::Identity(); }

// CONSTRUCTORS

Pose::Pose() {
  this->m_translation = Eigen::Vector3d::Zero();
  this->m_rotation = Eigen::Quaterniond::Identity();
}

Pose::Pose(const Eigen::Affine3d &pose) {
  this->m_translation = pose.translation();
  this->m_rotation = Eigen::Quaterniond(pose.rotation());
}

Pose::Pose(const std::array<double, 16> &pose)
    : Pose(array2eigen<4, 4>(pose)) {}

Pose::Pose(const Eigen::Matrix4d &pose) {
  Eigen::Affine3d affine_pose = Eigen::Affine3d(pose);
  this->m_translation = affine_pose.translation();
  this->m_rotation = Eigen::Quaterniond(affine_pose.rotation());
}

Pose::Pose(const Eigen::Matrix3d &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = Eigen::Quaterniond(rotation);
}

Pose::Pose(const Eigen::Vector4d &quaternion,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = Eigen::Quaterniond(quaternion);
}

Pose::Pose(const Eigen::Quaterniond &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = rotation;
}

Pose::Pose(const RPY &rotation, const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation =
      Eigen::AngleAxisd(rotation.roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(rotation.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rotation.yaw, Eigen::Vector3d::UnitZ());
}

Pose::Pose(const Eigen::Vector3d &rotation,
           const Eigen::Vector3d &translation) {
  this->m_translation = translation;
  this->m_rotation = Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ());
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
      this->m_rotation.toRotationMatrix().eulerAngles(0, 1, 2);
  return RPY{rpy_vec.x(), rpy_vec.y(), rpy_vec.z()};
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
      pose_b.rotation_m() * this->translation() + pose_b.translation();
  Eigen::Quaterniond rot = this->m_rotation * pose_b.m_rotation;
  return Pose(rot, trans);
}

Pose Pose::inverse() const {
  return Pose(this->m_rotation.inverse(), -this->m_translation);
}

bool Pose::is_close(const Pose &other, double eps_r, double eps_t) const {
  return (this->translation() - other.translation()).norm() < eps_t &&
         this->quaternion().angularDistance(other.quaternion()) < eps_r;
}

}  // namespace common
}  // namespace rcs
