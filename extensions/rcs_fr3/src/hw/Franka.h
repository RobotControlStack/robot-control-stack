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

#include "rcs/Controller.h"
#include "rcs/ControllerConfig.h"
#include "rcs/FR3Controllers.h"
#include "rcs/Kinematics.h"
#include "rcs/PinocchioDynamics.h"
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
  FrankaConfig() { this->robot_platform = common::RobotPlatform::HARDWARE; }
  std::string ip;
  IKSolver ik_solver = IKSolver::rcs_ik;
  double speed_factor = DEFAULT_SPEED_FACTOR;
  // Rate (Hz) at which set-point commands are streamed from the Python side.
  // Used to size the interpolation window between successive targets.
  int policy_rate = 20;
  // period of the control loop in seconds; libfranka runs it at 1 kHz, so
  // anything else is rejected
  double timestep = 0.001;
  // The gains and torque limit below are for the default controllers, used
  // when RobotConfig::controller is not set. A configured controller needs
  // async_control.
  // values from deoxys/config/joint-impedance-controller.yml
  common::Vector7d kp =
      (common::Vector7d() << 100., 100., 100., 100., 75., 150., 50.).finished();
  common::Vector7d kd =
      (common::Vector7d() << 20., 20., 20., 20., 7.5, 15.0, 5.0).finished();
  // FR3/Panda joint torque limits (Nm) from the Franka robot specification
  common::Vector7d torque_limit =
      (common::Vector7d() << 87., 87., 87., 87., 12., 12., 12.).finished();
  bool allow_high_collision = false;
  // values from deoxys/config/osc-position-controller.yml
  Eigen::Vector3d kp_p = (Eigen::Vector3d() << 150., 150., 150.).finished();
  double kp_r = 250.0;
  std::optional<FrankaLoad> load_parameters = std::nullopt;
  std::optional<common::Pose> world_to_robot = std::nullopt;
  common::Pose tcp_offset = common::Pose::Identity();
  // Indicates that Cartesian control uses tcp_offset.
  bool tcp_offset_explicit = false;
  bool async_control = false;
  // When true, on every (re)start of the joint controller the controller first
  // holds the current measured position (~zero error) and ramps its target to
  // the first commanded target over a gap-scaled window, blocking until it has
  // converged before streaming resumes. Avoids a torque/velocity jump when
  // (re)starting far from the target (e.g. after a PD-gain switch).
  bool blocking_move_on_start = false;
  // Max joint speed (rad/s) used to size the blocking approach window on
  // (re)start of the joint controller:
  // approach_time = max|q_target - q_now| / approach_joint_speed (clamped).
  // Only used when blocking_move_on_start is true.
  double approach_joint_speed = 0.4;
  // Max Cartesian translation (m/s) and rotation (rad/s) speeds used to size
  // the blocking approach window on (re)start of the OSC controller:
  // approach_time = max(trans_gap / approach_cartesian_speed,
  //                     rot_gap / approach_rotation_speed) (clamped).
  // Only used when blocking_move_on_start is true.
  double approach_cartesian_speed = 0.1;
  double approach_rotation_speed = 0.5;
  bool ignore_realtime = false;
  // robot_type, dof, joint_limits etc. are those of RobotConfig, set them per
  // robot (in Python with rcs.ROBOTS[robot_type].apply(cfg))
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
  double controller_time = 0.0;
  // The shared torque controllers (core), stepped in the libfranka control
  // loop: RobotConfig::controller, or without it FR3JointController and
  // FR3OSC with the gains below. They are created when a controller thread
  // starts, so the gains and the interpolation window (1 / policy_rate) stay
  // fixed for its lifetime.
  std::shared_ptr<common::PinocchioDynamics> dynamics;
  std::shared_ptr<common::JointController> torque_joint_controller;
  std::shared_ptr<common::CartesianController> torque_cartesian_controller;
  common::ControllerPair make_torque_controller(bool cartesian);
  common::ThreadSafeValue<franka::RobotState> curr_state;
  // guards the controllers between set_goal() and the control loop
  std::mutex interpolator_mutex;
  std::atomic<Controller> running_controller{Controller::none};
  common::ThreadSafeValue<std::exception_ptr> background_exception;
  void osc();
  void joint_controller();
  void create_dynamics();
  common::ControllerState controller_state(const franka::RobotState& state,
                                           double dt);
  void zero_torque_controller();
  void check_for_background_errors();
  void clear_background_error();

 public:
  Franka(const FrankaConfig& cfg,
         std::optional<std::shared_ptr<common::Kinematics>> ik = std::nullopt);
  ~Franka() override;

  bool set_config(const FrankaConfig& cfg);

  FrankaConfig* get_config() override;

  FrankaState* get_state() override;

  void set_default_robot_behavior();

  common::Pose get_cartesian_position() override;

  common::Pose get_cartesian_flange_position() override;

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
