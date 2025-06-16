#ifndef RCS_IK_H
#define RCS_IK_H

#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>

#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

struct RL {
  std::shared_ptr<rl::mdl::Model> mdl;
  std::shared_ptr<rl::mdl::Kinematic> kin;
  std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
};

class IK {
 private:
  RL rl;

 public:
  IK(const std::string& urdf_path, size_t max_duration_ms = 300);
  std::optional<VectorXd> ik(const Pose& pose, const VectorXd& q0,
                             const Pose& tcp_offset = Pose::Identity());
  Pose forward(const VectorXd& q0, const Pose& tcp_offset = Pose::Identity());

  // TODO: set max time
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H