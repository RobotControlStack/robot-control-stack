#ifndef RCS_FRANKA_H
#define RCS_FRANKA_H

#include <franka/robot.h>
#include <franka/robot_state.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "rcs/Kinematics.h"
#include "simadaptor.h"
#include "rcs/LinearPoseTrajInterpolator.h"
#include "rcs/Pose.h"
#include "rcs/Robot.h"
#include "rcs/utils.h"

namespace rcs {
namespace hw {

struct TAMHistorySample {
  double t;
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> tau_cmd;
  std::array<double, 7> gravity;
};

const double DEFAULT_SPEED_FACTOR = 0.2;
// 4 s of 1 kHz samples: the history-encoder thread polls at ~5 Hz and
// tolerates late polls without losing stream continuity (200 rows gave
// the Python side a zero-margin 0.2 s window).
const size_t TAM_HISTORY_SIZE = 4000;

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
  // Rate (Hz) at which set-point commands are streamed from the Python side.
  // Used to size the interpolation window between successive targets.
  int policy_rate = 20;
  // values from deoxys/config/joint-impedance-controller.yml
  common::Vector7d kp =
      (common::Vector7d() << 100., 100., 100., 100., 75., 150., 50.).finished();
  common::Vector7d kd =
      (common::Vector7d() << 20., 20., 20., 20., 7.5, 15.0, 5.0).finished();
  common::Vector7d torque_limit = common::Vector7d::Constant(5.0);
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
  bool tam_enabled = false;
  // Per-joint |clip| of the TAM residual torque in Nm before it is added to
  // the controller torque (wrist joints have a 12 Nm limit; keep headroom).
  common::Vector7d tam_residual_clip =
      (common::Vector7d() << 10., 10., 10., 10., 2., 2., 2.).finished();
  bool ignore_realtime = false;
  // >0: best-effort SCHED_FIFO at this priority for the async control
  // thread. Works on stock kernels when the rtprio rlimit allows it (unlike
  // ignore_realtime=false, which requires a PREEMPT_RT kernel); on failure
  // the thread keeps the normal scheduler and a warning is printed. The
  // TAM residual adds ~0.1-0.3 ms per 1 kHz tick, which misses deadlines
  // on a loaded non-RT machine without this.
  int rt_priority = 0;
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
  // Snapshot of m_cfg.policy_rate taken when a controller thread is started, so
  // the interpolation window stays consistent for the controller's lifetime.
  int m_active_policy_rate = 20;
  common::LinearJointPositionTrajInterpolator joint_interpolator;
  common::ThreadSafeValue<franka::RobotState> curr_state;
  std::mutex interpolator_mutex;
  std::atomic<Controller> running_controller{Controller::none};
  common::ThreadSafeValue<std::exception_ptr> background_exception;
  common::ThreadSafeValue<Eigen::VectorXd> tam_latent;
  common::ThreadSafeValue<std::optional<Eigen::VectorXd>> tam_mlp_weight{
      std::nullopt};
  common::ThreadSafeFixedBuffer<TAMHistorySample> tam_history{TAM_HISTORY_SIZE};
  // Parsed TAM MLP (set_tam_mlp_weight parses off the control thread).
  common::ThreadSafeValue<std::shared_ptr<const adaptor::SimAdaptor>> tam_model;
  // Robot-lifetime monotonic epoch for TAM history timestamps: controller
  // restarts (e.g. gain changes) must not restart the encoder's timeline —
  // they only leave a short, maskable gap in an otherwise continuous stream.
  const std::chrono::steady_clock::time_point tam_epoch =
      std::chrono::steady_clock::now();
  double tam_now() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         tam_epoch).count();
  }
  // Control-thread-only: ticks since the residual became active (1 s ramp).
  int tam_active_ticks = 0;
  void osc();
  void joint_controller();
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

  // Parses the packed TAM adaptor binary (one byte per vector element) and
  // publishes it to the control thread; throws std::runtime_error on a
  // malformed buffer. Parsing happens here, off the 1 kHz thread.
  void set_tam_mlp_weight(const Eigen::VectorXd& weight);
  void set_tam_latent(const Eigen::VectorXd& latent) {
    this->tam_latent.store(latent);
  }

  std::vector<TAMHistorySample> get_tam_history() {
    return this->tam_history.to_vector();
  }

  common::Vector7d tam_forward(const std::array<double, 7>& tau,
                               const franka::RobotState& robot_state,
                               const std::array<double, 7>& gravity);

  void reset() override;
  void close() override {};
};
}  // namespace hw
}  // namespace rcs

#endif  // RCS_FRANKA_H
