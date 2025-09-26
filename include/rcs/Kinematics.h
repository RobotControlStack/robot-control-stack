#ifndef RCS_IK_H
#define RCS_IK_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>

#include "Pose.h"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "utils.h"

namespace rcs {
namespace common {

class Kinematics {
 public:
  virtual ~Kinematics(){};
  virtual std::optional<VectorXd> inverse(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) = 0;
  virtual Pose forward(const VectorXd& q0, const Pose& tcp_offset) = 0;
};



class Pin : public Kinematics {
 private:
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;
  int FRAME_ID;

  pinocchio::Model model;
  pinocchio::Data data;

 public:
  Pin(const std::string& path, const std::string& frame_id, bool urdf);
  std::optional<VectorXd> inverse(
      const Pose& pose, const VectorXd& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
  Pose forward(const VectorXd& q0, const Pose& tcp_offset) override;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H