#ifndef RCS_FR3_H
#define RCS_FR3_H

#include <common/IK.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <franka/robot.h>

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
  std::optional<common::Pose> world_to_robot = std::nullopt;
  common::Pose tcp_offset = common::Pose::Identity();
};

struct FR3State : common::RState {};

class FR3 : public common::Robot {
 private:
  franka::Robot robot;
  FR3Config cfg;
  std::optional<std::shared_ptr<common::IK>> m_ik;
  std::string m_ip;

 public:
  FR3(const std::string &ip,
      std::optional<std::shared_ptr<common::IK>> ik = std::nullopt,
      const std::optional<FR3Config> &cfg = std::nullopt);
  ~FR3() override;

  std::string get_ip();

  bool set_parameters(const FR3Config &cfg);

  FR3Config *get_parameters() override;

  FR3State *get_state() override;

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

  std::optional<std::shared_ptr<common::IK>> get_ik() override;

  void set_cartesian_position_internal(const common::Pose &pose,
                                       double max_time,
                                       std::optional<double> elbow,
                                       std::optional<double> max_force = 5);

  void set_cartesian_position_ik(const common::Pose &x);

  common::Pose get_base_pose_in_world_coordinates() override;

  void reset() override;
};
}  // namespace hw
}  // namespace rcs

#endif  // RCS_FR3_H
