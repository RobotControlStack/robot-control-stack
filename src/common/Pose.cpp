#include "Pose.h"

namespace rcs {
namespace common {

// CONSTRUCTORS

// for python
Pose::Pose() {
  this->_translation = Eigen::Vector3d::Zero();
  this->_rotation = Eigen::Quaterniond::Identity();
}

Pose::Pose(const Eigen::Affine3d &pose) {
  this->_translation = pose.translation();
  this->_rotation = Eigen::Quaterniond(pose.rotation());
}

// for python
Pose::Pose(const Eigen::Vector3d &translation,
           const Eigen::Matrix3d &rotation) {
  this->_translation = translation;
  this->_rotation = Eigen::Quaterniond(rotation);
}

// for python
Pose::Pose(const Eigen::Vector3d &translation,
           const Eigen::Vector4d &rotation) {
  this->_translation = translation;
  this->_rotation = Eigen::Quaterniond(rotation);
}

Pose::Pose(const Eigen::Vector3d &translation,
           const Eigen::Quaterniond &rotation) {
  this->_translation = translation;
  this->_rotation = rotation;
}

Pose::Pose(const Eigen::Vector3d &translation, const RPY &rpy) {
  this->_translation = translation;
  this->_rotation = Eigen::AngleAxisd(rpy.roll, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(rpy.pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy.yaw, Eigen::Vector3d::UnitZ());
}

// GETTERS

// for python
Eigen::Vector3d Pose::translation() const { return this->_translation; }

// for python
Eigen::Matrix3d Pose::rotation_m() const {
  return this->_rotation.toRotationMatrix();
}

// for python
Eigen::Vector4d Pose::rotation_q() const { return this->_rotation.coeffs(); }

Eigen::Quaterniond Pose::quaternion() const { return this->_rotation; }

Eigen::Affine3d Pose::affine_matrix() const {
  Eigen::Affine3d pose =
      Eigen::Translation3d(this->_translation) * this->_rotation;
  return pose;
}

// for python
Eigen::Matrix4d Pose::pose_matrix() const {
  return this->affine_matrix().matrix();
}

std::array<double, 16> Pose::affine_array() const {
  std::array<double, 16> pose_array;
  Eigen::Matrix3d::Map(pose_array.data()) = this->affine_matrix().matrix();
}

RPY Pose::rpy() const {
  // ZYX, roll, pitch, yaw
  Eigen::Vector3d rpy_vec =
      this->_rotation.toRotationMatrix().eulerAngles(0, 1, 2);
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
  Pose result_pose(pos_result, quat_result);
  return result_pose;
}

std::string Pose::str() const {
  std::stringstream ss;

  RPY angles = this->rpy();

  ss << this->affine_matrix().matrix() << std::endl;
  ss << "roll: " << angles.roll << "\tpitch: " << angles.pitch
     << "\tyaw: " << angles.yaw;
  return ss.str();
}

Pose Pose::operator*(const Pose &pose_b) const {
  Eigen::Vector3d trans =
      pose_b.rotation_m() * this->translation() + pose_b.translation();
  Eigen::Quaterniond rot = this->_rotation * pose_b._rotation;
  return Pose(trans, rot);
}

}  // namespace common
}  // namespace rcs