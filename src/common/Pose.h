#ifndef RCS_POSE_H
#define RCS_POSE_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>

#include "utils.h"

namespace rcs {
namespace common {

Eigen::Vector3d IdentityTranslation();
Eigen::Matrix3d IdentityRotation();

struct RPY {
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  RPY(double roll = 0, double pitch = 0, double yaw = 0)
      : roll(roll), pitch(pitch), yaw(yaw) {}
  std::string str() const {
    return "RPY(" + std::to_string(roll) + ", " + std::to_string(pitch) + ", " +
           std::to_string(yaw) + ")";
  }
  RPY operator+(const RPY &rpy_b) const {
    return RPY{roll + rpy_b.roll, pitch + rpy_b.pitch, yaw + rpy_b.yaw};
  }
  Eigen::Matrix3d rotation_matrix() const {
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return rotation;
  }
  Eigen::Vector3d as_vector() const {
    Eigen::Vector3d rotation(
        (Eigen::Vector3d() << roll, pitch, yaw).finished());
    return rotation;
  }

  bool is_close(const RPY &other, double eps = 1e-8) const {
    return this->as_vector().isApprox(other.as_vector(), eps);
  }
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
   * Pose.
   * For python bindings.
   */
  Pose();

  Pose(const Eigen::Affine3d &pose);

  Pose(const std::array<double, 16> &pose);

  /**
   * @brief Construct a new Pose object from a 4x4 matrix.
   * For python bindings.
   *
   * @param pose
   */
  Pose(const Eigen::Matrix4d &pose);

  /**
   * @brief Construct a new Pose object from a 3x3 rotation matrix and a 3D
   * translation vector.
   * For python bindings.
   *
   * @param rotation
   * @param translation
   */
  Pose(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);

  /**
   * @brief Construct a new Pose object from a 4D quaternion and a 3D
   * translation vector.
   * For python bindings.
   *
   * @param rotation
   * @param translation
   */
  Pose(const Eigen::Vector4d &rotation, const Eigen::Vector3d &translation);

  Pose(const Eigen::Quaterniond &quaternion,
       const Eigen::Vector3d &translation);

  /**
   * @brief Construct a new Pose object from a RPY struct and a 3D translation
   * vector.
   * For python bindings.
   *
   * @param rotation
   * @param translation
   */
  Pose(const RPY &rotation, const Eigen::Vector3d &translation);

  /**
   * @brief Construct a new Pose object from a 3D RPY and a 3D translation
   * vector.
   * For python bindings.
   *
   * @param rotation
   * @param translation
   */
  Pose(const Eigen::Vector3d &rotation, const Eigen::Vector3d &translation);

  // GETTERS

  /**
   * @brief returns the translational part of the transformation. It is similar
   * to the getPosition function
   * For python bindings.
   *
   * @return 3D vector of the translation
   */
  Eigen::Vector3d translation() const;

  /**
   * @brief returns the rotational part of the pose as a matrix
   * For python bindings.
   *
   * @return 3x3 rotation matrix
   */
  Eigen::Matrix3d rotation_m() const;

  /**
   * @brief returns the rotational part of the pose as quaternion
   * For python bindings.
   *
   * @return 4D vector of the quaternion
   */
  Eigen::Vector4d rotation_q() const;

  /**
   * @brief returns the rotational part of the pose as quaternion
   * @return Rotation as quaternion
   */
  Eigen::Quaterniond quaternion() const;

  Eigen::Affine3d affine_matrix() const;

  /**
   * @brief Returns the pose as a 4x4 matrix
   * For python bindings.
   *
   * @return 4x4 matrix
   */
  Eigen::Matrix4d pose_matrix() const;

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
   * For python bindings.
   *
   * @return RPY struct with roll, pitch and yaw
   */
  RPY rotation_rpy() const;

  /**
   * @brief Interpolates the Pose to a destination Pose
   * For python bindings.
   *
   * @param dest_pose goal Pose
   * @param progress value of [0-1] where 0 is the start pose and 1 is the end
   * Pose
   * @return interpolated Pose
   */
  Pose interpolate(const Pose &dest_pose, double progress) const;

  /**
   * @brief Returns the XYZRPY representation of the Pose
   * For python bindings.
   */
  Vector6d xyzrpy() const;

  /**
   * @brief Converts a Pose to a String
   * For python bindings.
   *
   * @return Pose as String
   */
  std::string str() const;

  /**
   * @brief Performs multiplication of Poses as  homogenous matrix
   * multiplication
   * For python bindings. TODO: look how to bind operator overloading
   * @param pose_b an other
   * @return  the result of the multiplication
   */
  Pose operator*(const Pose &pose_b) const;

  /**
   * @brief Returns the inverse of the Pose
   * For python bindings.
   *
   * @return Inverse Pose
   */
  Pose inverse() const;

  /**
   * @brief Checks if two Poses are equal within a certain epsilon
   *
   * @param other Pose to compare
   * @param eps epsilon
   *
   * @return true if the Poses are equal
   */
  bool is_close(const Pose &other, double eps_r = 1e-8,
                double eps_t = 1e-8) const;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_POSE_H