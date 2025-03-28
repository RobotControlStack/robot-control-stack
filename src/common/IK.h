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

class IK {
 public:
  virtual ~IK(){};
  virtual std::optional<Vector7d> ik(
      const Pose& pose, const Vector7d& q0,
      const Pose& tcp_offset = Pose::Identity()) = 0;
};

class RL : public IK {
 private:
  struct RLData {
    std::shared_ptr<rl::mdl::Model> mdl;
    std::shared_ptr<rl::mdl::Kinematic> kin;
    std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
  };
  RLData rl_data;

 public:
  RL(const std::string& urdf_path, size_t max_duration_ms = 300);
  ~RL() override = default;
  std::optional<Vector7d> ik(
      const Pose& pose, const Vector7d& q0,
      const Pose& tcp_offset = Pose::Identity()) override;
};
}  // namespace common
}  // namespace rcs

#endif  // RCS_IK_H