#ifndef RCS_IK_H
#define RCS_IK_H

#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <pinocchio/parsers/urdf.hpp>

#include "Pose.h"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "utils.h"

namespace rcs {
namespace common {

class IK {
 public:
  virtual ~IK(){};
  virtual std::optional<Vector7d> ik(
      const Pose& pose, const Vector7d& q0,
      const Pose& tcp_offset = Pose::Identity()) = 0;
};

class RL : public IK {
 private:
  const int random_restarts = 0;
  const double eps = 1e-3;
  struct {
    std::shared_ptr<rl::mdl::Model> mdl;
    std::shared_ptr<rl::mdl::Kinematic> kin;
    std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
  } rl_data;

 public:
  RL(const std::string& urdf_path, size_t max_duration_ms = 300);
  std::optional<Vector7d> ik(
      const Pose& pose, const Vector7d& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
};

class Pin : public IK {
 private:
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  const int JOINT_ID = 7;

  pinocchio::Model model;
  pinocchio::Data data;

 public:
  Pin(const std::string& urdf_path);
  std::optional<Vector7d> ik(
      const Pose& pose, const Vector7d& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H