#ifndef RCS_FR3_H
#define RCS_FR3_H

#include <common/IK.h>
#include <common/LinearPoseTrajInterpolator.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <franka/robot.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace rcs {
namespace hw {

const double DEFAULT_SPEED_FACTOR = 0.2;

struct FR3Load {
  double load_mass;
  std::optional<Eigen::Vector3d> f_x_cload;
  std::optional<Eigen::Matrix3d> load_inertia;
};
enum IKSolver { franka = 0, rcs };
// modes: joint-space control, operational-space control, zero-torque
// control
enum Controller { none = 0, jsc, osc, ztc };
struct FR3Config : common::RobotConfig {
  // TODO: max force and elbow?
  // TODO: we can either write specific bindings for each, or we use python
  // dictionaries with these objects
  IKSolver ik_solver = IKSolver::franka;
  double speed_factor = DEFAULT_SPEED_FACTOR;
  std::optional<FR3Load> load_parameters = std::nullopt;
  std::optional<common::Pose> nominal_end_effector_frame = std::nullopt;
  std::optional<common::Pose> world_to_robot = std::nullopt;
  common::Pose tcp_offset = common::Pose::Identity();
  bool async_control = false;
};

struct FR3State : common::RobotState {};

class FR3 : public common::Robot {
 private:
  franka::Robot robot;
  FR3Config cfg;
  std::optional<std::shared_ptr<common::IK>> m_ik;
  std::optional<std::thread> control_thread = std::nullopt;
  common::LinearPoseTrajInterpolator traj_interpolator;
  double controller_time = 0.0;
  common::LinearJointPositionTrajInterpolator joint_interpolator;
  franka::RobotState curr_state;
  std::mutex interpolator_mutex;
  Controller running_controller = Controller::none;
  void osc();
  void joint_controller();
  void zero_torque_controller();

 public:
  FR3(const std::string &ip,
      std::optional<std::shared_ptr<common::IK>> ik = std::nullopt,
      const std::optional<FR3Config> &cfg = std::nullopt);
  ~FR3() override;

  bool set_parameters(const FR3Config &cfg);

  FR3Config *get_parameters() override;

  FR3State *get_state() override;

  void set_default_robot_behavior();

  common::Pose get_cartesian_position() override;

  void set_joint_position(const common::VectorXd &q) override;

  common::VectorXd get_joint_position() override;

  void set_guiding_mode(bool x, bool y, bool z, bool roll, bool pitch, bool yaw,
                        bool elbow);

  void controller_set_joint_position(const common::Vector7d &desired_q);
  void osc_set_cartesian_position(
      const common::Pose &desired_pose_EE_in_base_frame);
  void zero_torque_guiding();

  void stop_control_thread();

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
