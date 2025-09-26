#ifndef RCS_RL_H
#define RCS_RL_H

#include <rcs/Kinematics.h>
#include <rcs/Pose.h>
#include <rcs/utils.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>

namespace rcs {
namespace robotics_library {

class RoboticsLibraryIK : public rcs::common::Kinematics {
 private:
  const int random_restarts = 0;
  const double eps = 1e-3;
  struct {
    std::shared_ptr<rl::mdl::Model> mdl;
    std::shared_ptr<rl::mdl::Kinematic> kin;
    std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
  } rl_data;

 public:
  RoboticsLibraryIK(const std::string& urdf_path, size_t max_duration_ms = 300)
      : rl_data() {
    this->rl_data.mdl = rl::mdl::UrdfFactory().create(urdf_path);
    this->rl_data.kin =
        std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rl_data.mdl);
    this->rl_data.ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(
        this->rl_data.kin.get());
    this->rl_data.ik->setRandomRestarts(this->random_restarts);
    this->rl_data.ik->setEpsilon(this->eps);
    this->rl_data.ik->setDuration(std::chrono::milliseconds(max_duration_ms));
  }
  std::optional<rcs::common::VectorXd> inverse(
      const rcs::common::Pose& pose, const rcs::common::VectorXd& q0,
      const rcs::common::Pose& tcp_offset =
          rcs::common::Pose::Identity()) override {
    // pose is assumed to be in the robots coordinate frame
    this->rl_data.kin->setPosition(q0);
    this->rl_data.kin->forwardPosition();
    rcs::common::Pose new_pose = pose * tcp_offset.inverse();

    this->rl_data.ik->addGoal(new_pose.affine_matrix(), 0);
    bool success = this->rl_data.ik->solve();
    if (success) {
      // is this forward needed and is it mabye possible to call
      // this on the model?
      this->rl_data.kin->forwardPosition();
      return this->rl_data.kin->getPosition();
    } else {
      return std::nullopt;
    }
  }
  rcs::common::Pose forward(const rcs::common::VectorXd& q0,
                            const rcs::common::Pose& tcp_offset) override {
    // pose is assumed to be in the robots coordinate frame
    this->rl_data.kin->setPosition(q0);
    this->rl_data.kin->forwardPosition();
    rcs::common::Pose pose = this->rl_data.kin->getOperationalPosition(0);
    // apply the tcp offset
    return pose * tcp_offset.inverse();
  }
};

}  // namespace robotics_library
}  // namespace rcs

#endif  // RCS_RL_H
