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
  Eigen::Vector3d _translation;
  Eigen::Quaterniond _rotation;

 public:
  // CONSTRUCTORS
  /**
   * Identity Affine Constructor. Generates an Affine3d with Identity Pose
   */
  Pose();

  Pose(const Eigen::Affine3d &pose);

  Pose(const Eigen::Vector3d &translation, const Eigen::Matrix3d &rotation);

  Pose(const Eigen::Vector3d &translation, const Eigen::Vector4d &rotation);

  Pose(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation);

  Pose(const Eigen::Vector3d &translation, const RPY &rpy);

  // GETTERS

  /**
   * returns the translational part of the transformation. It is similar to the
   * getPosition function
   * @return
   */
  Eigen::Vector3d translation() const;

  /**
   * returns the rotational part of the pose as a matrix
   * @return 3x3 rotation matrix
   */
  Eigen::Matrix3d rotation_m() const;

  /**
   * returns the rotational part of the pose as quaternion
   * @return
   */
  Eigen::Vector4d rotation_q() const;

  /**
   * returns the rotational part of the pose as quaternion
   * @return
   */
  Eigen::Quaterniond quaternion() const;

  Eigen::Affine3d affine_matrix() const;

  Eigen::Matrix4d Pose::pose_matrix() const;

  /**
   * Returns the affine transformation in matrix form
   * in a flattened std::array in column major order
   * this is useful for libfranka
   */
  std::array<double, 16> affine_array() const;

  RPY Pose::rpy() const;

  /**
   * Interpolates the Pose to a destination Pose
   * @param dest_pose goal Pose
   * @param progress value of [0-1] where 0 is the start pose and 1 is the end
   * Pose
   * @return interpolated Pose
   */
  Pose interpolate(const Pose &dest_trans, double progress) const;

  /**
   * Converts a Pose to a String
   * @return Pose as String
   */
  std::string str() const;

  /**
   * Performs multiplication of Poses as  homogenous matrix multiplication
   * @param pose_b an other
   * @return  the result of the multiplication
   */
  Pose operator*(const Pose &pose_b) const;

  // for python
  static Pose Identity() { return Pose(); };
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_POSE_H