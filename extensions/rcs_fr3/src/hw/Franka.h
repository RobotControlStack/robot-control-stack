#ifndef RCS_FRANKA_H
#define RCS_FRANKA_H

#include <franka/robot.h>
#include <franka/robot_state.h>

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rcs/Kinematics.h"
#include "rcs/LinearPoseTrajInterpolator.h"
#include "rcs/Pose.h"
#include "rcs/Robot.h"
#include "rcs/utils.h"

namespace rcs {
namespace hw {

const double DEFAULT_SPEED_FACTOR = 0.2;

struct FrankaLoad {
  double load_mass;
  std::optional<Eigen::Vector3d> f_x_cload;
  std::optional<Eigen::Matrix3d> load_inertia;
};
enum IKSolver { franka_ik = 0, rcs_ik };
// modes: joint-space control, operational-space control, zero-torque
// control
enum Controller { none = 0, jsc, osc, ztc };
struct FrankaConfig : common::RobotConfig {
  std::string ip;
  common::RobotType robot_type = common::RobotType::FR3;
  common::RobotPlatform robot_platform = common::RobotPlatform::HARDWARE;
  IKSolver ik_solver = IKSolver::rcs_ik;
  double speed_factor = DEFAULT_SPEED_FACTOR;
  std::optional<FrankaLoad> load_parameters = std::nullopt;
  std::optional<common::Pose> nominal_end_effector_frame = std::nullopt;
  std::optional<common::Pose> world_to_robot = std::nullopt;
  bool async_control = false;
  bool tcp_offset_configured_in_desk = true;
  bool ignore_realtime = false;
  size_t dof = 7;
  Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor> joint_limits =
      (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
           // low 7‐tuple
           -2.3093,
       -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895,
       // high 7‐tuple
       2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895)
          .finished();
};

struct FR3Config : FrankaConfig {};
struct PandaConfig : FrankaConfig {
  common::RobotType robot_type = common::RobotType::Panda;
  Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor> joint_limits =
      (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
           // low 7‐tuple
           -166. / 180. * M_PI,
       -101. / 180. * M_PI, -166. / 180. * M_PI, -176. / 180. * M_PI,
       -166. / 180. * M_PI, -1. / 180. * M_PI, -166. / 180. * M_PI,
       // high 7‐tuple
       166. / 180. * M_PI, 101. / 180. * M_PI, 166. / 180. * M_PI,
       -4. / 180. * M_PI, 166. / 180. * M_PI, 215. / 180. * M_PI,
       166. / 180. * M_PI)
          .finished();
};

struct FrankaState : common::RobotState {
  franka::RobotState robot_state;
};

class Franka : public common::Robot {
 private:
  franka::Robot robot;
  FrankaConfig m_cfg;
  std::optional<std::shared_ptr<common::Kinematics>> m_ik;
  std::optional<std::thread> control_thread = std::nullopt;
  common::LinearPoseTrajInterpolator traj_interpolator;
  double controller_time = 0.0;
  common::LinearJointPositionTrajInterpolator joint_interpolator;
  franka::RobotState curr_state;
  std::mutex interpolator_mutex;
  std::atomic<Controller> running_controller{Controller::none};
  std::exception_ptr background_exception = nullptr;
  std::mutex exception_mutex;
  void osc();
  void joint_controller();
  void zero_torque_controller();
  void check_for_background_errors();

 public:
  Franka(const FrankaConfig& cfg,
         std::optional<std::shared_ptr<common::Kinematics>> ik = std::nullopt);
  ~Franka() override;

  bool set_config(const FrankaConfig& cfg);

  FrankaConfig* get_config() override;

  FrankaState* get_state() override;

  void set_default_robot_behavior();

  common::Pose get_cartesian_position() override;

  void set_joint_position(const common::VectorXd& q) override;

  common::VectorXd get_joint_position() override;

  void set_guiding_mode(bool x, bool y, bool z, bool roll, bool pitch, bool yaw,
                        bool elbow);

  void controller_set_joint_position(const common::Vector7d& desired_q);
  void osc_set_cartesian_position(
      const common::Pose& desired_pose_EE_in_base_frame);
  void zero_torque_guiding();

  void stop_control_thread();

  void move_home() override;

  void automatic_error_recovery();

  static void wait_milliseconds(int milliseconds);

  void double_tap_robot_to_continue();

  void set_cartesian_position(const common::Pose& pose) override;

  std::optional<std::shared_ptr<common::Kinematics>> get_ik() override;

  void set_cartesian_position_internal(const common::Pose& pose,
                                       double max_time,
                                       std::optional<double> elbow,
                                       std::optional<double> max_force = 5);

  void set_cartesian_position_ik(const common::Pose& x);

  common::Pose get_base_pose_in_world_coordinates() override;

  void reset() override;
  void close() override {};
};
}  // namespace hw
}  // namespace rcs

#endif  // RCS_FRANKA_H
