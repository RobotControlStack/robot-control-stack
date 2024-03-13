#ifndef RCS_FR3_H
#define RCS_FR3_H

#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <franka/robot.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/UrdfFactory.h>

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

namespace rcs {
namespace hw {

const common::Vector7d q_home((common::Vector7d() << 0, -M_PI_4, 0, -3 * M_PI_4,
                               0, M_PI_2, M_PI_4)
                                  .finished());
const double DEFAULT_SPEED_FACTOR = 0.2;

struct FR3Load {
  double load_mass;
  std::optional<Eigen::Vector3d> f_x_cload;
  std::optional<Eigen::Matrix3d> load_inertia;
};
enum IKController { internal = 0, robotics_library };
struct FR3Config : common::RConfig {
  // TODO: max force and elbow?
  // TODO: we can either write specific bindings for each, or we use python
  // dictionaries with these objects
  IKController controller = IKController::internal;
  bool guiding_mode_enabled = true;
  double speed_factor = DEFAULT_SPEED_FACTOR;
  std::optional<FR3Load> load_parameters = std::nullopt;
  std::optional<common::Pose> nominal_end_effector_frame = std::nullopt;
};

struct FR3State : common::RState {};

class FR3 : public common::Robot {
 private:
  franka::Robot robot;
  rl::mdl::Dynamic model;
  std::optional<std::unique_ptr<rl::mdl::JacobianInverseKinematics>> ik;
  FR3Config cfg;

 public:
  FR3(const std::string &ip,
      const std::optional<std::string> &filename = std::nullopt,
      const std::optional<FR3Config> &cfg = std::nullopt);
  ~FR3() override;

  bool set_parameters(const common::RConfig &cfg) override;

  std::unique_ptr<common::RConfig> get_parameters() override;

  std::unique_ptr<common::RState> get_state() override;

  void set_default_robot_behavior();

  common::Pose get_cartesian_position() override;

  void set_joint_position(const common::Vector7d &q) override;

  common::Vector7d get_joint_position() override;

  void set_guiding_mode(bool enabled);

  void move_home() override;

  void automatic_error_recovery();

  static void wait_milliseconds(int milliseconds);

  void double_tap_robot_to_continue();

  void set_cartesian_position(const common::Pose &pose) override;

  void set_cartesian_position_internal(const common::Pose &pose,
                                       double max_time,
                                       std::optional<double> elbow,
                                       std::optional<double> max_force = 5);

  void set_cartesian_position_rl(const common::Pose &x);
};
}  // namespace hw
}  // namespace rcs

#endif  // RCS_FR3_H