#ifndef RCS_FRANKA_H
#define RCS_FRANKA_H

#include <franka/robot.h>
#include <franka/robot_state.h>

#include <atomic>
#include <deque>
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

// Per-tick diagnostic record for debugging the TAM integration. Filled on the
// 1 kHz control thread into a preallocated ring (no allocation, no I/O in the
// loop) and dumped to CSV off the control thread via dump_tam_debug_log().
struct TAMDebugSample {
  double t = 0.0;             // tam_now() timestamp (s, robot-lifetime clock)
  double period = 0.0;        // libfranka control period (s)
  double compute_us = 0.0;    // controller compute time this tick (us)
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> desired_q{};        // interpolated joint target
  std::array<double, 7> tau_pd{};           // controller torque BEFORE TAM
  std::array<double, 7> delta_raw{};        // TAM residual pre-clip, pre-ramp
  std::array<double, 7> delta_clipped{};    // TAM residual post-clip, pre-ramp
  std::array<double, 7> delta_applied{};    // TAM residual actually added (post-ramp)
  std::array<double, 7> tau_post_tam{};     // tau_pd + delta_applied (pre rate-limit)
  std::array<double, 7> tau_final{};        // after limitRate + safety clip (applied & recorded)
  std::array<double, 7> gravity{};          // model.gravity(robot_state)
  std::array<double, 7> tau_meas{};         // robot_state.tau_J (measured link torque)
  double latent_norm = 0.0;                 // ||latent|| used this tick
  double latent_age = 0.0;                  // s since the latent was last set
  int latent_size = 0;
  double ramp = 0.0;                        // residual ramp scalar [0,1]
  int active_ticks = 0;
  double window_dt = 0.0;                   // t(now) - t(oldest row in MLP window)
  int history_steps = 0;
  double max_ood_tau = 0.0;                 // max |z-score| of now-row torque vs norm_stats
  // bit0: model present, bit1: MLP window valid, bit2: forward_stream active
  int flags = 0;
};

// Control-thread-only scratch that tam_forward fills each tick so the control
// loop can log what went in and out of the module. No synchronization needed:
// tam_forward is only ever called from the control thread.
struct TAMForwardDiag {
  bool model_present = false;
  bool window_ok = false;
  bool active = false;
  int history_steps = 0;
  double window_dt = 0.0;
  double latent_age = 0.0;
  double latent_norm = 0.0;
  int latent_size = 0;
  double ramp = 0.0;
  int active_ticks = 0;
  double max_ood_tau = 0.0;
  Eigen::Matrix<double, 7, 1> delta_raw = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> delta_clipped = Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> delta_applied = Eigen::Matrix<double, 7, 1>::Zero();
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
  // Apply franka::limitRate to the final commanded torque. On by default to
  // satisfy the FCI torque-rate reflex. Can be disabled to test whether the
  // rate limiter (which sits inside the TAM feedback loop and distorts the
  // applied-torque history TAM conditions on) is the source of oscillation.
  // NOTE: off usually trips FCI because PD+residual can sum past the reflex.
  bool rate_limit = true;
  // What TAM records as the "applied torque" history. When true, record the
  // pre-rate-limit intended torque (tau_pd + residual) instead of the
  // post-limiter command, so the rate limiter (needed for FCI) no longer
  // distorts the history TAM conditions on -- keeping TAM's feedback consistent
  // with training while the robot still receives a smooth command.
  bool tam_history_pre_ratelimit = false;
  bool ignore_realtime = false;
  // Best-effort real-time scheduling for the async control thread at this
  // priority (0 disables): SCHED_FIFO when the rtprio rlimit allows it,
  // otherwise SCHED_RR via RealtimeKit, otherwise a warning and the normal
  // scheduler. Works on stock kernels (unlike ignore_realtime=false, which
  // requires PREEMPT_RT). On by default: the 1 kHz torque loop always
  // benefits, and the TAM residual's ~0.1-0.3 ms per tick misses deadlines
  // on a loaded non-RT machine without it.
  int rt_priority = 80;
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
  // Control-thread-only ring of the newest history samples so tam_forward
  // reads its MLP window without touching the shared buffer's mutex (the
  // control thread is the only writer of both).
  std::deque<TAMHistorySample> tam_recent;
  // Wall time (tam_now() clock) at which the most recent latent was set, so the
  // control thread can log how stale the latent is. -1 until the first latent.
  std::atomic<double> tam_latent_set_time{-1.0};
  std::atomic<double> tam_latent_norm{0.0};
  // Control-thread-only scratch filled by tam_forward each tick.
  TAMForwardDiag last_tam_diag;
  // Debug ring buffer (preallocated; single writer = control thread). When
  // logging is on the control loop overwrites the oldest slot, so after a
  // failure the buffer holds the most recent tam_debug_capacity ticks.
  std::atomic<bool> tam_logging{false};
  size_t tam_debug_capacity = 120000;  // ~2 min at 1 kHz
  std::vector<TAMDebugSample> tam_debug_log;
  size_t tam_debug_head = 0;   // next slot to write
  bool tam_debug_full = false; // whether the ring has wrapped
  void tam_debug_record(const TAMDebugSample& s);
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
    this->tam_latent_norm.store(latent.norm());
    this->tam_latent_set_time.store(this->tam_now());
  }

  std::vector<TAMHistorySample> get_tam_history() {
    return this->tam_history.to_vector();
  }

  common::Vector7d tam_forward(const std::array<double, 7>& tau,
                               const franka::RobotState& robot_state,
                               const std::array<double, 7>& gravity);

  // Offline self-test: run the TAM adaptor MLP on a fully static window and the
  // given latent, print and return the residual. Uses ONLY the passed inputs
  // (ignores live robot_state / tam_recent) and does NOT command the robot — for
  // validating the C++ adaptor against the Python reference on known inputs.
  // q/qd/tau_cmd/gravity are each [history_steps x 7]; tau_model is built as
  // tau_cmd + gravity per row (matching the live path). Returns the RAW residual
  // (pre-clip, pre-ramp); the clipped residual is also printed. Requires
  // set_tam_mlp_weight() first; throws on shape/latent mismatch.
  common::Vector7d tam_forward_test(const Eigen::MatrixXd& q,
                                    const Eigen::MatrixXd& qd,
                                    const Eigen::MatrixXd& tau_cmd,
                                    const Eigen::MatrixXd& gravity,
                                    const Eigen::VectorXd& latent);

  // Number of history rows the loaded TAM adaptor consumes (0 if no model set),
  // and the expected latent length. Useful to slice test windows correctly.
  int tam_history_steps() const {
    const auto m = this->tam_model.load();
    return m ? m->history_steps : 0;
  }
  int tam_latent_dim() const {
    const auto m = this->tam_model.load();
    return m ? m->expected_history_embedding_cols() : 0;
  }

  // TAM debugging: enable/disable per-tick recording, clear the ring, and dump
  // it to CSV. dump/clear must be called with the control thread stopped.
  void set_tam_logging(bool on) {
    if (on && this->tam_debug_log.size() != this->tam_debug_capacity) {
      // Preallocate off the control thread so the 1 kHz loop never allocates.
      this->tam_debug_log.assign(this->tam_debug_capacity, TAMDebugSample());
      this->tam_debug_head = 0;
      this->tam_debug_full = false;
    }
    this->tam_logging.store(on);
  }
  void clear_tam_debug_log() {
    this->tam_debug_head = 0;
    this->tam_debug_full = false;
  }
  // Writes the recorded ticks (chronological order) to a CSV file. Returns the
  // number of rows written.
  size_t dump_tam_debug_log(const std::string& path);

  void reset() override;
  void close() override {};
};
}  // namespace hw
}  // namespace rcs

#endif  // RCS_FRANKA_H
