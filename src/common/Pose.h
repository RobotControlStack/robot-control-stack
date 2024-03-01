#ifndef RCS_POSE_H
#define RCS_POSE_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>

namespace rcs {
namespace common {

Eigen::Vector3d IdentityTranslation() { return Eigen::Vector3d::Zero(); }
Eigen::Matrix3d IdentityRotation() { return Eigen::Matrix3d::Identity(); }

struct RPY {
  double roll;
  double pitch;
  double yaw;
};

/**
 * Immutable abstraction class with python bindings for affine 3D transformation
 * to access them confidently from python
 */
class Pose {
 private:
  Eigen::Vector3d m_translation;
  Eigen::Quaterniond m_rotation;

 public:
  // STATIC FUNCTIONS

  /**
   * @brief Returns the Identity Pose, which is a Pose with zero translation and
   * identity rotation. This is useful for default values.
   *
   * @return Pose
   */
  static Pose Identity() { return Pose(); };

  // CONSTRUCTORS

  /**
   * @brief Identity Affine Constructor. Generates an Affine3d with Identity
   * Pose
   */
  Pose();

  Pose(const Eigen::Affine3d &pose);

  Pose(const std::array<double, 16> &pose);

  Pose(const Eigen::Matrix4d &pose);

  Pose(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);

  Pose(const Eigen::Vector4d &rotation, const Eigen::Vector3d &translation);

  Pose(const Eigen::Quaterniond &rotation, const Eigen::Vector3d &translation);

  Pose(const RPY &rpy, const Eigen::Vector3d &translation);

  // GETTERS

  /**
   * @brief returns the translational part of the transformation. It is similar
   * to the getPosition function
   * @return 3D vector of the translation
   */
  Eigen::Vector3d translation() const;

  /**
   * @brief returns the rotational part of the pose as a matrix
   * @return 3x3 rotation matrix
   */
  Eigen::Matrix3d rotation_m() const;

  /**
   * @brief returns the rotational part of the pose as quaternion
   * @return 4D vector of the quaternion
   */
  Eigen::Vector4d rotation_q() const;

  /**
   * @brief returns the rotational part of the pose as quaternion
   * @return Rotation as quaternion
   */
  Eigen::Quaterniond quaternion() const;

  Eigen::Affine3d affine_matrix() const;

  Eigen::Matrix4d Pose::pose_matrix() const;

  /**
   * @brief Returns the affine transformation in matrix form
   * in a flattened std::array in column major order
   * this is useful for libfranka
   *
   * @return 4x4 matrix as a flattened array
   */
  std::array<double, 16> affine_array() const;

  /**
   * @brief Returns the RPY representation of the rotation
   * @return RPY struct with roll, pitch and yaw
   */
  RPY Pose::rpy() const;

  /**
   * @brief Interpolates the Pose to a destination Pose
   * @param dest_pose goal Pose
   * @param progress value of [0-1] where 0 is the start pose and 1 is the end
   * Pose
   * @return interpolated Pose
   */
  Pose interpolate(const Pose &dest_pose, double progress) const;

  /**
   * @brief Converts a Pose to a String
   * @return Pose as String
   */
  std::string str() const;

  /**
   * @brief Performs multiplication of Poses as  homogenous matrix
   * multiplication
   * @param pose_b an other
   * @return  the result of the multiplication
   */
  Pose operator*(const Pose &pose_b) const;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_POSE_H