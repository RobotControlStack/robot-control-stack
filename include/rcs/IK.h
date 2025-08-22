#ifndef RCS_IK_H
#define RCS_IK_H

#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <pinocchio/parsers/urdf.hpp>
#include <string>

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
  virtual std::optional<VectorXd> ik(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) = 0;
  virtual Pose forward(const VectorXd& q0, const Pose& tcp_offset) = 0;
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
  RL(const std::string& path, size_t max_duration_ms = 300);
  std::optional<VectorXd> ik(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
  Pose forward(const VectorXd& q0, const Pose& tcp_offset) override;
};

class Pin : public IK {
 private:
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  const double eps_pos = 0.01;
  const double eps_ori = 10 * (M_PI / 180.0);
  int FRAME_ID;

  pinocchio::Model model;
  pinocchio::Data data;

 public:
  Pin(const std::string& path, const std::string& frame_id, bool urdf);
  std::optional<VectorXd> ik(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
  Pose forward(const VectorXd& q0, const Pose& tcp_offset) override;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H