#include "Franka.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "FrankaMotionGenerator.h"
#include "rcs/Pose.h"

namespace rcs {
namespace hw {

common::Pose GetFlangeInBaseFrame(const franka::RobotState& robot_state) {
  return common::Pose(robot_state.O_T_EE) *
         common::Pose(robot_state.F_T_EE).inverse();
}

common::Pose GetTCPInBaseFrame(const franka::RobotState& robot_state,
                               const common::Pose& tcp_offset) {
  return GetFlangeInBaseFrame(robot_state) * tcp_offset;
}

Franka::Franka(const FrankaConfig& cfg,
               std::optional<std::shared_ptr<common::Kinematics>> ik)
    : m_cfg(cfg),
      m_ik(ik),
      robot(cfg.ip, cfg.ignore_realtime ? franka::RealtimeConfig::kIgnore
                                        : franka::RealtimeConfig::kEnforce) {
  // set collision behavior and impedance
  this->set_default_robot_behavior();
  this->set_guiding_mode(true, true, true, true, true, true, true);
}

Franka::~Franka() {
  try {
    this->stop_control_thread();
  } catch (const franka::Exception& e) {
    std::cerr << "Exception in ~Franka(): " << e.what() << std::endl;
  }
}

/**
 * @brief Set the parameters for the robot
 * @param cfg The configuration for the robot, it should be a FrankaConfig type
 * otherwise the call will fail
 */
bool Franka::set_config(const FrankaConfig& cfg) {
  this->m_cfg = cfg;
  this->m_cfg.speed_factor = std::min(std::max(cfg.speed_factor, 0.0), 1.0);

  if (this->m_cfg.load_parameters.has_value()) {
    auto load_value = &(this->m_cfg.load_parameters.value());
    if (!load_value->f_x_cload.has_value()) {
      load_value->f_x_cload = Eigen::Vector3d::Zero();
    }
    if (!load_value->load_inertia.has_value()) {
      load_value->load_inertia = Eigen::Matrix3d::Zero();
    }

    this->robot.setLoad(
        load_value->load_mass,
        common::eigen2array<3, 1>(load_value->f_x_cload.value()),
        common::eigen2array<3, 3>(load_value->load_inertia.value()));
  }
  return true;
}

FrankaConfig* Franka::get_config() {
  // copy config to heap
  FrankaConfig* cfg = new FrankaConfig();
  *cfg = this->m_cfg;
  return cfg;
}

FrankaState* Franka::get_state() {
  franka::RobotState current_robot_state;
  if (this->running_controller.load() == Controller::none) {
    current_robot_state = this->robot.readOnce();
  } else {
    current_robot_state = this->curr_state.load();
  }
  auto* state = new FrankaState();
  state->robot_state = current_robot_state;
  return state;
}

void Franka::set_tam_mlp_weight(const Eigen::VectorXd& weight) {
  this->tam_mlp_weight.store(weight);
  // The vector carries the packed TAM adaptor binary, one byte per element
  // (the format written by SimAdaptorInference.export_simadaptor_weights_cpp;
  // it includes the MLP architecture and the input normalization statistics).
  // Parse here, off the 1 kHz control thread, and publish the parsed model.
  std::string bytes(static_cast<size_t>(weight.size()), '\0');
  for (Eigen::Index i = 0; i < weight.size(); ++i) {
    const double v = weight(i);
    if (!(v >= 0.0 && v <= 255.0)) {
      throw std::runtime_error(
          "set_tam_mlp_weight: element " + std::to_string(i) +
          " is not a byte value; pass the .bin file bytes as float64");
    }
    bytes[static_cast<size_t>(i)] = static_cast<char>(static_cast<uint8_t>(v));
  }
  std::shared_ptr<const adaptor::SimAdaptor> model =
      adaptor::SimAdaptor::LoadFromMemory(bytes.data(), bytes.size());
  if (!model) {
    throw std::runtime_error(
        "set_tam_mlp_weight: could not parse the TAM adaptor binary");
  }
  if (model->dof != 7) {
    throw std::runtime_error("set_tam_mlp_weight: expected dof=7, got " +
                             std::to_string(model->dof));
  }
  this->tam_model.store(model);
}

common::Vector7d Franka::tam_forward(const std::array<double, 7>& tau,
                                     const franka::RobotState& robot_state,
                                     const std::array<double, 7>& gravity) {
  const std::shared_ptr<const adaptor::SimAdaptor> model =
      this->tam_model.load();
  common::Vector7d delta = common::Vector7d::Zero();
  common::Vector7d delta_raw = common::Vector7d::Zero();
  // Reset this tick's diagnostic scratch (control-thread only).
  TAMForwardDiag& diag = this->last_tam_diag;
  diag = TAMForwardDiag();
  diag.model_present = static_cast<bool>(model);
  const double latent_set_t = this->tam_latent_set_time.load();
  diag.latent_norm = this->tam_latent_norm.load();
  diag.latent_size = static_cast<int>(this->tam_latent.load().size());
  diag.latent_age = latent_set_t < 0.0 ? -1.0 : (this->tam_now() - latent_set_t);
  std::vector<adaptor::SimAdaptor::StreamRow> rows;
  if (model) {
    diag.history_steps = model->history_steps;
    // The MLP conditions on the last history_steps samples in the
    // ideal-model (gravity-included) torque space: history_steps-1 recorded
    // ticks plus the current one from the arguments. The newest recorded row
    // must be contiguous with now — right after a controller restart the
    // buffer still ends before the restart gap (see tam_now()).
    const int T = model->history_steps;
    const std::deque<TAMHistorySample>& past = this->tam_recent;
    if (past.size() + 1 >= static_cast<size_t>(T) &&
        this->tam_now() - past.back().t <= 0.05) {
      rows.reserve(static_cast<size_t>(T));
      for (auto it = past.end() - static_cast<long>(T - 1); it != past.end();
           ++it) {
        const TAMHistorySample& s = *it;
        adaptor::SimAdaptor::StreamRow r;
        for (int j = 0; j < 7; ++j) {
          r.q[j] = s.q[j];
          r.dq[j] = s.dq[j];
          r.tau_model[j] = s.tau_cmd[j] + s.gravity[j];
        }
        rows.push_back(r);
      }
      adaptor::SimAdaptor::StreamRow now;
      for (int j = 0; j < 7; ++j) {
        now.q[j] = robot_state.q[j];
        now.dq[j] = robot_state.dq[j];
        now.tau_model[j] = tau[j] + gravity[j];
      }
      rows.push_back(now);
      diag.window_ok = true;
      diag.window_dt =
          (T > 1) ? (this->tam_now() -
                     past[past.size() - static_cast<size_t>(T - 1)].t)
                  : 0.0;
      diag.max_ood_tau = model->now_row_tau_ood(now);
    }
  }


  const bool active =
      !rows.empty() && model->forward_stream(rows, this->tam_latent.load(),
                                             this->m_cfg.tam_residual_clip,
                                             delta, &delta_raw);
  diag.active = active;
  if (!active) {
    this->tam_active_ticks = 0;
    diag.active_ticks = 0;
    diag.ramp = 0.0;
    return common::Vector7d::Zero();
  }
  // Ramp the residual in over ~1 s whenever it (re)activates so enabling
  // TAM (or the first latent) never steps the torque.
  this->tam_active_ticks = std::min(this->tam_active_ticks + 1, 1000);
  const double ramp = static_cast<double>(this->tam_active_ticks) / 1000.0;
  const common::Vector7d applied = ramp * delta;
  diag.ramp = ramp;
  diag.active_ticks = this->tam_active_ticks;
  diag.delta_raw = delta_raw;
  diag.delta_clipped = delta;
  diag.delta_applied = applied;
  // print applied
  std::cout << "[tam] applied: " << applied.transpose() << std::endl;
  return applied;
}

common::Vector7d Franka::tam_forward_test(const Eigen::MatrixXd& q,
                                          const Eigen::MatrixXd& qd,
                                          const Eigen::MatrixXd& tau_cmd,
                                          const Eigen::MatrixXd& gravity,
                                          const Eigen::VectorXd& latent) {
  const std::shared_ptr<const adaptor::SimAdaptor> model = this->tam_model.load();
  if (!model) {
    throw std::runtime_error(
        "tam_forward_test: no TAM model set; call set_tam_mlp_weight() first");
  }
  const int T = model->history_steps;
  const int R = static_cast<int>(q.rows());
  auto check = [&](const Eigen::MatrixXd& m, const char* name) {
    if (m.rows() != R || m.cols() != 7) {
      throw std::runtime_error(
          std::string("tam_forward_test: ") + name +
          " must be [R x 7] with R matching q; got [" + std::to_string(m.rows()) +
          " x " + std::to_string(m.cols()) + "], q has " + std::to_string(R) +
          " rows");
    }
  };
  check(qd, "qd");
  check(tau_cmd, "tau_cmd");
  check(gravity, "gravity");
  if (R < T) {
    throw std::runtime_error(
        "tam_forward_test: need at least history_steps=" + std::to_string(T) +
        " rows, got " + std::to_string(R));
  }
  if (latent.size() != model->expected_history_embedding_cols()) {
    throw std::runtime_error(
        "tam_forward_test: latent size " + std::to_string(latent.size()) +
        " != expected " + std::to_string(model->expected_history_embedding_cols()));
  }
  if (R != T) {
    std::cout << "[tam_test] window has " << R << " rows; model history_steps="
              << T << " -> using the LAST " << T << " rows (matches live path)"
              << std::endl;
  }

  // Take the last T rows (as the live path takes the last history_steps of the
  // recent buffer). Build model-space torque as tau_cmd + gravity per row (pass
  // gravity=0 if tau_cmd is already model-space).
  const int off = R - T;
  std::vector<adaptor::SimAdaptor::StreamRow> rows(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    const int r = off + t;
    for (int j = 0; j < 7; ++j) {
      rows[static_cast<size_t>(t)].q[j] = q(r, j);
      rows[static_cast<size_t>(t)].dq[j] = qd(r, j);
      rows[static_cast<size_t>(t)].tau_model[j] = tau_cmd(r, j) + gravity(r, j);
    }
  }

  common::Vector7d delta = common::Vector7d::Zero();
  common::Vector7d raw = common::Vector7d::Zero();
  const bool ok = model->forward_stream(rows, latent, this->m_cfg.tam_residual_clip,
                                        delta, &raw);
  if (!ok) {
    throw std::runtime_error(
        "tam_forward_test: forward_stream failed (history_steps / latent size / "
        "internal shape mismatch)");
  }
  std::cout << std::setprecision(9);
  std::cout << "[tam_test] raw residual (pre-clip, pre-ramp): " << raw.transpose()
            << std::endl;
  std::cout << "[tam_test] clipped residual                 : " << delta.transpose()
            << std::endl;
  // Return the RAW residual: the pure adaptor-MLP output to compare against the
  // Python reference. (This does NOT touch the control loop or the robot.)
  return raw;
}

void Franka::tam_debug_record(const TAMDebugSample& s) {
  // Single writer (control thread). Preallocated ring; overwrite oldest.
  if (this->tam_debug_log.empty()) return;
  this->tam_debug_log[this->tam_debug_head] = s;
  this->tam_debug_head = (this->tam_debug_head + 1) % this->tam_debug_log.size();
  if (this->tam_debug_head == 0) this->tam_debug_full = true;
}

size_t Franka::dump_tam_debug_log(const std::string& path) {
  // Must be called with the control thread stopped (no concurrent writer).
  const size_t cap = this->tam_debug_log.size();
  const size_t n = this->tam_debug_full ? cap : this->tam_debug_head;
  if (n == 0) {
    std::cerr << "[tam] dump_tam_debug_log: no samples recorded" << std::endl;
    return 0;
  }
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("dump_tam_debug_log: cannot open " + path);
  }
  out << "t,period,compute_us,latent_norm,latent_age,latent_size,ramp,"
         "active_ticks,window_dt,history_steps,max_ood_tau,model_present,"
         "window_ok,active";
  const char* names[] = {"q",           "dq",         "desired_q",
                         "tau_pd",      "delta_raw",  "delta_clipped",
                         "delta_applied", "tau_post_tam", "tau_final",
                         "gravity",     "tau_meas"};
  for (const char* nm : names) {
    for (int j = 0; j < 7; ++j) out << "," << nm << j;
  }
  out << "\n";
  out << std::setprecision(9);
  const size_t start = this->tam_debug_full ? this->tam_debug_head : 0;
  for (size_t k = 0; k < n; ++k) {
    const TAMDebugSample& s = this->tam_debug_log[(start + k) % cap];
    out << s.t << "," << s.period << "," << s.compute_us << "," << s.latent_norm
        << "," << s.latent_age << "," << s.latent_size << "," << s.ramp << ","
        << s.active_ticks << "," << s.window_dt << "," << s.history_steps << ","
        << s.max_ood_tau << "," << ((s.flags & 1) ? 1 : 0) << ","
        << ((s.flags & 2) ? 1 : 0) << "," << ((s.flags & 4) ? 1 : 0);
    const std::array<const std::array<double, 7>*, 11> vecs = {
        &s.q,           &s.dq,         &s.desired_q,   &s.tau_pd,
        &s.delta_raw,   &s.delta_clipped, &s.delta_applied, &s.tau_post_tam,
        &s.tau_final,   &s.gravity,    &s.tau_meas};
    for (const auto* v : vecs) {
      for (int j = 0; j < 7; ++j) out << "," << (*v)[j];
    }
    out << "\n";
  }
  out.close();
  std::cerr << "[tam] wrote " << n << " rows to " << path << std::endl;
  return n;
}

void Franka::set_default_robot_behavior() {
  this->robot.setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                   {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                   {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                   {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                   {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                   {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                   {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                   {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  this->robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  this->robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

common::Pose Franka::get_cartesian_position() {
  this->check_for_background_errors();
  franka::RobotState robot_state;
  if (this->running_controller.load() == Controller::none) {
    robot_state = this->robot.readOnce();
    this->curr_state.store(robot_state);
  } else {
    robot_state = this->curr_state.load();
  }
  return GetTCPInBaseFrame(robot_state, this->m_cfg.tcp_offset);
}

common::Pose Franka::get_cartesian_flange_position() {
  this->check_for_background_errors();
  franka::RobotState robot_state;
  if (this->running_controller.load() == Controller::none) {
    robot_state = this->robot.readOnce();
    this->curr_state.store(robot_state);
  } else {
    robot_state = this->curr_state.load();
  }
  return GetFlangeInBaseFrame(robot_state);
}

void Franka::set_joint_position(const common::VectorXd& q) {
  if (this->m_cfg.async_control) {
    this->controller_set_joint_position(q);
    return;
  }
  // sync control
  FrankaMotionGenerator motion_generator(this->m_cfg.speed_factor, q);
  this->robot.control(motion_generator);
}

common::VectorXd Franka::get_joint_position() {
  this->check_for_background_errors();
  franka::RobotState robot_state;
  if (this->running_controller.load() == Controller::none) {
    robot_state = this->robot.readOnce();
    this->curr_state.store(robot_state);
  } else {
    robot_state = this->curr_state.load();
  }
  return common::Vector7d(robot_state.q.data());
}

void Franka::set_guiding_mode(bool x, bool y, bool z, bool roll, bool pitch,
                              bool yaw, bool elbow) {
  std::array<bool, 6> activated = {x, y, z, roll, pitch, yaw};
  this->robot.setGuidingMode(activated, elbow);
}

void PInverse(const Eigen::MatrixXd& M, Eigen::MatrixXd& M_inv,
              double epsilon = 0.00025) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_vals =
      svd.singularValues();

  Eigen::MatrixXd S_inv = M;
  S_inv.setZero();
  for (int i = 0; i < singular_vals.size(); i++) {
    if (singular_vals(i) < epsilon) {
      S_inv(i, i) = 0.;
    } else {
      S_inv(i, i) = 1. / singular_vals(i);
    }
  }
  M_inv = Eigen::MatrixXd(svd.matrixV() * S_inv * svd.matrixU().transpose());
}

void TorqueSafetyGuardFn(std::array<double, 7>& tau_d_array,
                         const common::Vector7d& torque_limit) {
  for (size_t i = 0; i < tau_d_array.size(); i++) {
    if (tau_d_array[i] < -torque_limit[i]) {
      tau_d_array[i] = -torque_limit[i];
    } else if (tau_d_array[i] > torque_limit[i]) {
      tau_d_array[i] = torque_limit[i];
    }
  }
}

void Franka::controller_set_joint_position(const common::Vector7d& desired_q) {
  this->check_for_background_errors();
  // from deoxys/config/osc-position-controller.yml
  double traj_interpolation_time_fraction = 1.0;  // in s
  // form deoxys/config/charmander.yml
  int traj_rate = 500;

  const bool starting_fresh =
      this->running_controller.load() == Controller::none;

  if (starting_fresh) {
    this->controller_time = 0.0;
    this->m_active_policy_rate = this->m_cfg.policy_rate;
    this->get_joint_position();
    this->joint_interpolator = common::LinearJointPositionTrajInterpolator();
  } else if (this->running_controller.load() != Controller::jsc) {
    // runtime error
    throw std::runtime_error(
        "Controller type must but joint space but is " +
        std::to_string(static_cast<int>(this->running_controller.load())) +
        ". To change controller type stop the current controller first.");
  } else {
    this->interpolator_mutex.lock();
  }

  franka::RobotState state_now = this->curr_state.load();
  const common::Vector7d q_now =
      Eigen::Map<common::Vector7d>(state_now.q.data());

  const bool approach_on_start =
      starting_fresh && this->m_cfg.blocking_move_on_start;
  double approach_time = -1.0;
  if (approach_on_start) {
    const double kMinApproachTime = 0.3;  // s
    const double kMaxApproachTime = 5.0;  // s
    const double max_gap = (desired_q - q_now).cwiseAbs().maxCoeff();
    const double speed = std::max(this->m_cfg.approach_joint_speed, 1e-6);
    approach_time =
        std::clamp(max_gap / speed, kMinApproachTime, kMaxApproachTime);
  }

  this->joint_interpolator.reset(
      this->controller_time, q_now, desired_q, this->m_active_policy_rate,
      traj_rate, traj_interpolation_time_fraction, approach_time);

  // if not thread is running, then start
  if (starting_fresh) {
    this->running_controller.store(Controller::jsc);
    this->control_thread = std::thread(&Franka::joint_controller, this);
  } else {
    this->interpolator_mutex.unlock();
  }

  if (approach_on_start) {
    // Block until the controller has driven the robot to desired_q (or a
    // safety timeout elapses).
    const double pos_tol = 0.02;  // rad
    const double vel_tol = 0.05;  // rad/s
    const double timeout = approach_time + 2.0;
    const auto start = std::chrono::steady_clock::now();
    while (true) {
      this->check_for_background_errors();
      franka::RobotState state = this->curr_state.load();
      const common::Vector7d q = Eigen::Map<common::Vector7d>(state.q.data());
      const common::Vector7d dq = Eigen::Map<common::Vector7d>(state.dq.data());
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      const double pos_err = (q - desired_q).cwiseAbs().maxCoeff();
      const double vel = dq.cwiseAbs().maxCoeff();
      if (elapsed >= approach_time && pos_err < pos_tol && vel < vel_tol) {
        break;
      }
      if (elapsed > timeout) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void Franka::check_for_background_errors() {
  std::exception_ptr ex = this->background_exception.load_and_clear();
  if (ex) {
    this->stop_control_thread();
    std::rethrow_exception(ex);
  }
}

void Franka::clear_background_error() {
  this->background_exception.store(nullptr);
}

void Franka::osc_set_cartesian_position(
    const common::Pose& desired_pose_EE_in_base_frame) {
  this->check_for_background_errors();
  // from deoxys/config/osc-position-controller.yml
  double traj_interpolation_time_fraction = 1.0;
  // form deoxys/config/charmander.yml
  int traj_rate = 500;

  const bool starting_fresh =
      this->running_controller.load() == Controller::none;

  if (starting_fresh) {
    this->controller_time = 0.0;
    this->m_active_policy_rate = this->m_cfg.policy_rate;
    this->curr_state.store(this->robot.readOnce());
    this->traj_interpolator = common::LinearPoseTrajInterpolator();
  } else if (this->running_controller.load() != Controller::osc) {
    throw std::runtime_error(
        "Controller type must but osc but is " +
        std::to_string(static_cast<int>(this->running_controller.load())) +
        ". To change controller type stop the current controller first.");
  } else {
    this->interpolator_mutex.lock();
  }

  common::Pose curr_pose =
      GetTCPInBaseFrame(this->curr_state.load(), this->m_cfg.tcp_offset);

  const bool approach_on_start =
      starting_fresh && this->m_cfg.blocking_move_on_start;
  double approach_time = -1.0;
  if (approach_on_start) {
    const double kMinApproachTime = 0.3;  // s
    const double kMaxApproachTime = 5.0;  // s
    const double trans_gap =
        (desired_pose_EE_in_base_frame.translation() - curr_pose.translation())
            .norm();
    double dot = std::abs(curr_pose.quaternion().normalized().dot(
        desired_pose_EE_in_base_frame.quaternion().normalized()));
    dot = std::min(1.0, dot);
    const double rot_gap = 2.0 * std::acos(dot);
    const double trans_speed =
        std::max(this->m_cfg.approach_cartesian_speed, 1e-6);
    const double rot_speed =
        std::max(this->m_cfg.approach_rotation_speed, 1e-6);
    approach_time =
        std::clamp(std::max(trans_gap / trans_speed, rot_gap / rot_speed),
                   kMinApproachTime, kMaxApproachTime);
  }

  this->traj_interpolator.reset(
      this->controller_time, curr_pose.translation(), curr_pose.quaternion(),
      desired_pose_EE_in_base_frame.translation(),
      desired_pose_EE_in_base_frame.quaternion(), this->m_active_policy_rate,
      traj_rate, traj_interpolation_time_fraction, approach_time);

  // if not thread is running, then start
  if (starting_fresh) {
    this->running_controller.store(Controller::osc);
    this->control_thread = std::thread(&Franka::osc, this);
  } else {
    this->interpolator_mutex.unlock();
  }

  if (approach_on_start) {
    // Block until the controller has driven the robot to the desired pose (or a
    // safety timeout elapses).
    const double pos_tol = 0.005;  // m
    const double ori_tol = 0.02;   // rad
    const double vel_tol = 0.05;   // rad/s (joint-space proxy for "stopped")
    const double timeout = approach_time + 2.0;
    const auto start = std::chrono::steady_clock::now();
    const Eigen::Vector3d target_p =
        desired_pose_EE_in_base_frame.translation();
    const Eigen::Quaterniond target_q =
        desired_pose_EE_in_base_frame.quaternion().normalized();
    while (true) {
      this->check_for_background_errors();
      franka::RobotState state = this->curr_state.load();
      const common::Vector7d dq = Eigen::Map<common::Vector7d>(state.dq.data());
      const common::Pose meas_pose =
          GetTCPInBaseFrame(state, this->m_cfg.tcp_offset);
      const double pos_err = (target_p - meas_pose.translation()).norm();
      double dot = std::abs(meas_pose.quaternion().normalized().dot(target_q));
      dot = std::min(1.0, dot);
      const double ori_err = 2.0 * std::acos(dot);
      const double vel = dq.cwiseAbs().maxCoeff();
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start)
                                 .count();
      if (elapsed >= approach_time && pos_err < pos_tol && ori_err < ori_tol &&
          vel < vel_tol) {
        break;
      }
      if (elapsed > timeout) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

// method to stop thread
void Franka::stop_control_thread() {
  if (this->control_thread.has_value()) {
    this->running_controller.store(Controller::none);
    if (this->control_thread->joinable()) {
      this->control_thread->join();
    }
    this->control_thread.reset();
  }
}

void Franka::osc() {
  adaptor::TryElevateControlThreadPriority(this->m_cfg.rt_priority);
  franka::Model model = this->robot.loadModel();
  const Eigen::Vector3d kp_p_cfg = this->m_cfg.kp_p;
  const double kp_r_cfg = this->m_cfg.kp_r;
  const common::Vector7d torque_limit = this->m_cfg.torque_limit;
  const bool allow_high_collision = this->m_cfg.allow_high_collision;

  this->controller_time = 0.0;
  // The TAM history buffer intentionally survives controller restarts:
  // timestamps come from a robot-lifetime monotonic clock, so a restart is
  // just a short gap in a continuous stream. Only the residual ramp starts
  // over (the gains may have changed).
  this->tam_active_ticks = 0;
  this->tam_ideal_gravity.reset();
  if (this->m_cfg.tam_enabled && !this->m_cfg.tam_ideal_model_path.empty()) {
    try {
      this->tam_ideal_gravity = std::make_unique<adaptor::IdealModelGravity>(
          this->m_cfg.tam_ideal_model_path);
      std::cerr << "[rcs] TAM: ideal-model gravity from "
                << this->m_cfg.tam_ideal_model_path << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[rcs] TAM: ideal-model gravity disabled (" << e.what()
                << "); TAM will use the robot's gravity model" << std::endl;
    }
  }

  // conservative collision and impedance behavior
  this->set_default_robot_behavior();

  if (allow_high_collision) {
    // High collision threshold values for high impedance.
    this->robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  }

  // from bench mark
  // ([150.0, 150.0, 60.0], 250.0), // kp_translation, kp_rotation
  // ([60.0, 150.0, 150.0], 250.0), // kd_translation, kd_rotation

  // from config file
  // Kp:
  // translation: [150.0, 150.0, 150.0]
  // rotation: 250.0

  Eigen::Matrix<double, 3, 3> Kp_p = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 3, 3> Kp_r = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 3, 3> Kd_p = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 3, 3> Kd_r = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 7, 1> static_q_task_;
  Eigen::Matrix<double, 7, 1> residual_mass_vec_;
  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;
  Eigen::Array<double, 7, 1> avoidance_weights_;

  Kp_p.diagonal() << kp_p_cfg;
  Kp_r.diagonal() << kp_r_cfg, kp_r_cfg, kp_r_cfg;

  Kd_p << Kp_p.cwiseSqrt() * 2.0;
  Kd_r << Kp_r.cwiseSqrt() * 2.0;

  static_q_task_ << 0.09017809387254755, -0.9824203501652151,
      0.030509718397568178, -2.694229634937343, 0.057700675144720104,
      1.860298714876101, 0.8713759453244422;

  // The manual residual mass matrix to add on the internal mass matrix
  residual_mass_vec_ << 0.0, 0.0, 0.0, 0.0, 0.1, 0.5, 0.5;

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  avoidance_weights_ << 1., 1., 1., 1., 1., 10., 10.;

  try {
    this->robot.control([&](const franka::RobotState& robot_state,
                            franka::Duration period) -> franka::Torques {
      std::chrono::high_resolution_clock::time_point t1 =
          std::chrono::high_resolution_clock::now();

      // torques handler
      if (this->running_controller.load() == Controller::none) {
        return franka::MotionFinished(franka::Torques(robot_state.tau_J_d));
      }
      // TO BE replaced
      // if (!this->control_thread_running && dq.maxCoeff() < 0.0001) {
      //   return franka::MotionFinished(franka::Torques(tau_d_array));
      // }

      Eigen::Vector3d desired_pos_EE_in_base_frame;
      Eigen::Quaterniond desired_quat_EE_in_base_frame;

      // form deoxys/config/charmander.yml
      int policy_rate = 20;
      int traj_rate = 500;

      this->curr_state.store(robot_state);

      this->interpolator_mutex.lock();
      this->controller_time += period.toSec();
      this->traj_interpolator.next_step(this->controller_time,
                                        desired_pos_EE_in_base_frame,
                                        desired_quat_EE_in_base_frame);
      this->interpolator_mutex.unlock();

      // end torques handler

      Eigen::Matrix<double, 7, 1> tau_d;

      std::array<double, 49> mass_array = model.mass(robot_state);
      Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

      M = M + Eigen::Matrix<double, 7, 7>(residual_mass_vec_.asDiagonal());

      // coriolis and gravity
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(
          coriolis_array.data());

      std::array<double, 7> gravity_array = model.gravity(robot_state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(
          gravity_array.data());

      std::array<double, 42> jacobian_array = model.zeroJacobian(
          franka::Frame::kEndEffector, robot_state.q,
          this->m_cfg.tcp_offset.affine_array(), robot_state.EE_T_K);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
          jacobian_array.data());

      Eigen::MatrixXd jacobian_pos(3, 7);
      Eigen::MatrixXd jacobian_ori(3, 7);
      jacobian_pos << jacobian.block(0, 0, 3, 7);
      jacobian_ori << jacobian.block(3, 0, 3, 7);

      // Express OSC feedback in the same TCP frame exposed by the public
      // Cartesian API.
      common::Pose T_EE_in_base_frame_pose =
          GetTCPInBaseFrame(robot_state, this->m_cfg.tcp_offset);
      Eigen::Affine3d T_EE_in_base_frame =
          T_EE_in_base_frame_pose.affine_matrix();

      Eigen::Vector3d pos_EE_in_base_frame(T_EE_in_base_frame.translation());
      Eigen::Quaterniond quat_EE_in_base_frame(T_EE_in_base_frame.linear());

      // Nullspace goal
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      // Joint velocity
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      if (desired_quat_EE_in_base_frame.coeffs().dot(
              quat_EE_in_base_frame.coeffs()) < 0.0) {
        quat_EE_in_base_frame.coeffs() << -quat_EE_in_base_frame.coeffs();
      }

      Eigen::Vector3d pos_error;

      pos_error << desired_pos_EE_in_base_frame - pos_EE_in_base_frame;
      Eigen::Quaterniond quat_error(desired_quat_EE_in_base_frame.inverse() *
                                    quat_EE_in_base_frame);
      Eigen::Vector3d ori_error;
      ori_error << quat_error.x(), quat_error.y(), quat_error.z();
      ori_error << -T_EE_in_base_frame.linear() * ori_error;

      // Compute matrices
      Eigen::Matrix<double, 7, 7> M_inv(M.inverse());
      Eigen::MatrixXd Lambda_inv(6, 6);
      Lambda_inv << jacobian * M_inv * jacobian.transpose();
      Eigen::MatrixXd Lambda(6, 6);
      PInverse(Lambda_inv, Lambda);

      Eigen::Matrix<double, 7, 6> J_inv;
      J_inv << M_inv * jacobian.transpose() * Lambda;
      Eigen::Matrix<double, 7, 7> Nullspace;
      Nullspace << Eigen::MatrixXd::Identity(7, 7) -
                       jacobian.transpose() * J_inv.transpose();

      // Decoupled mass matrices
      Eigen::MatrixXd Lambda_pos_inv(3, 3);
      Lambda_pos_inv << jacobian_pos * M_inv * jacobian_pos.transpose();
      Eigen::MatrixXd Lambda_ori_inv(3, 3);
      Lambda_ori_inv << jacobian_ori * M_inv * jacobian_ori.transpose();

      Eigen::MatrixXd Lambda_pos(3, 3);
      Eigen::MatrixXd Lambda_ori(3, 3);
      PInverse(Lambda_pos_inv, Lambda_pos);
      PInverse(Lambda_ori_inv, Lambda_ori);

      pos_error = pos_error.unaryExpr(
          [](double x) { return (abs(x) < 1e-4) ? 0. : x; });
      ori_error = ori_error.unaryExpr(
          [](double x) { return (abs(x) < 5e-3) ? 0. : x; });

      tau_d << jacobian_pos.transpose() *
                       (Lambda_pos *
                        (Kp_p * pos_error - Kd_p * (jacobian_pos * dq))) +
                   jacobian_ori.transpose() *
                       (Lambda_ori *
                        (Kp_r * ori_error - Kd_r * (jacobian_ori * dq)));

      // nullspace control
      tau_d << tau_d + Nullspace * (static_q_task_ - q);

      // Add joint avoidance potential
      Eigen::Matrix<double, 7, 1> avoidance_force;
      avoidance_force.setZero();
      Eigen::Matrix<double, 7, 1> dist2joint_max;
      Eigen::Matrix<double, 7, 1> dist2joint_min;

      dist2joint_max = joint_max_.matrix() - q;
      dist2joint_min = q - joint_min_.matrix();

      for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.25 && dist2joint_max[i] > 0.1)
          avoidance_force[i] += -avoidance_weights_[i] * dist2joint_max[i];
        if (dist2joint_min[i] < 0.25 && dist2joint_min[i] > 0.1)
          avoidance_force[i] += avoidance_weights_[i] * dist2joint_min[i];
      }
      tau_d << tau_d + Nullspace * avoidance_force;
      for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.) tau_d[i] = 0.;
        if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.) tau_d[i] = 0.;
      }

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // end of controller
      std::chrono::high_resolution_clock::time_point t2 =
          std::chrono::high_resolution_clock::now();
      auto time =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

      if (this->m_cfg.tam_enabled && this->tam_ideal_gravity) {
        // Let the plant see the ideal model's gravity compensation (TAM's
        // training convention): the robot adds its own model gravity
        // internally, so command the difference. The TAM history then
        // records tau_cmd + g_robot = PD + g_ideal + residual, truthfully.
        const std::array<double, 7> g_ideal =
            this->tam_ideal_gravity->gravity(robot_state.q);
        for (int j = 0; j < 7; ++j) {
          tau_d_array[j] += g_ideal[j] - gravity_array[j];
        }
      }
      if (this->m_cfg.tam_enabled) {
        Eigen::VectorXd::Map(&tau_d_array[0], 7) +=
            tam_forward(tau_d_array, robot_state, gravity_array);
      }
      // Intended (pre-rate-limit) commanded torque, optionally recorded into the
      // TAM history so the rate limiter does not distort TAM's conditioning.
      const std::array<double, 7> tau_pre_ratelimit = tau_d_array;

      std::array<double, 7> tau_d_rate_limited =
          this->m_cfg.rate_limit
              ? franka::limitRate(franka::kMaxTorqueRate, tau_d_array,
                                  robot_state.tau_J_d)
              : tau_d_array;

      TorqueSafetyGuardFn(tau_d_rate_limited, torque_limit);

      if (this->m_cfg.tam_enabled) {
        // safe q, q_dot and tau
        const TAMHistorySample sample{.t = this->tam_now(),
                                      .q = robot_state.q,
                                      .dq = robot_state.dq,
                                      .tau_cmd = this->m_cfg.tam_history_pre_ratelimit
                                                     ? tau_pre_ratelimit
                                                     : tau_d_rate_limited,
                                      .gravity = gravity_array};
        this->tam_history.push_back(sample);
        this->tam_recent.push_back(sample);
        if (this->tam_recent.size() > 32) {
          this->tam_recent.pop_front();
        }
      }

      return tau_d_rate_limited;
    });
  } catch (...) {
    this->background_exception.store(std::current_exception());
  }

  // Ensure we mark the controller as stopped so we can restart later
  this->running_controller.store(Controller::none);
}

void Franka::joint_controller() {
  adaptor::TryElevateControlThreadPriority(this->m_cfg.rt_priority);
  franka::Model model = this->robot.loadModel();
  const common::Vector7d Kp = this->m_cfg.kp;
  const common::Vector7d Kd = this->m_cfg.kd;
  const common::Vector7d torque_limit = this->m_cfg.torque_limit;
  const bool allow_high_collision = this->m_cfg.allow_high_collision;
  this->controller_time = 0.0;
  // The TAM history buffer intentionally survives controller restarts:
  // timestamps come from a robot-lifetime monotonic clock, so a restart is
  // just a short gap in a continuous stream. Only the residual ramp starts
  // over (the gains may have changed).
  this->tam_active_ticks = 0;
  this->tam_ideal_gravity.reset();
  if (this->m_cfg.tam_enabled && !this->m_cfg.tam_ideal_model_path.empty()) {
    try {
      this->tam_ideal_gravity = std::make_unique<adaptor::IdealModelGravity>(
          this->m_cfg.tam_ideal_model_path);
      std::cerr << "[rcs] TAM: ideal-model gravity from "
                << this->m_cfg.tam_ideal_model_path << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "[rcs] TAM: ideal-model gravity disabled (" << e.what()
                << "); TAM will use the robot's gravity model" << std::endl;
    }
  }

  // conservative collision and impedance behavior
  this->set_default_robot_behavior();

  if (allow_high_collision) {
    // High collision threshold values for high impedance.
    this->robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  }

  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  try {
    this->robot.control([&](const franka::RobotState& robot_state,
                            franka::Duration period) -> franka::Torques {
      std::chrono::high_resolution_clock::time_point t1 =
          std::chrono::high_resolution_clock::now();

      // torques handler
      if (this->running_controller.load() == Controller::none) {
        return franka::MotionFinished(franka::Torques(robot_state.tau_J_d));
      }

      common::Vector7d desired_q;

      this->curr_state.store(robot_state);

      this->interpolator_mutex.lock();
      this->controller_time += period.toSec();
      this->joint_interpolator.next_step(this->controller_time, desired_q);
      this->interpolator_mutex.unlock();
      // end torques handler

      Eigen::Matrix<double, 7, 1> tau_d;

      // Current joint velocity
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

      // Current joint position
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      Eigen::MatrixXd joint_pos_error(7, 1);

      joint_pos_error << desired_q - q;

      tau_d << Kp.cwiseProduct(joint_pos_error) - Kd.cwiseProduct(dq);

      Eigen::Matrix<double, 7, 1> dist2joint_max;
      Eigen::Matrix<double, 7, 1> dist2joint_min;

      dist2joint_max = joint_max_.matrix() - q;
      dist2joint_min = q - joint_min_.matrix();

      for (int i = 0; i < 7; i++) {
        if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.) tau_d[i] = 0.;
        if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.) tau_d[i] = 0.;
      }

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      // end of controller
      std::chrono::high_resolution_clock::time_point t2 =
          std::chrono::high_resolution_clock::now();
      auto time =
          std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

      // Controller torque before the TAM residual (for logging).
      const std::array<double, 7> tau_pd_array = tau_d_array;

      std::array<double, 7> gravity_array = model.gravity(robot_state);
      if (this->m_cfg.tam_enabled && this->tam_ideal_gravity) {
        // Let the plant see the ideal model's gravity compensation (TAM's
        // training convention): the robot adds its own model gravity
        // internally, so command the difference. The TAM history then
        // records tau_cmd + g_robot = PD + g_ideal + residual, truthfully.
        const std::array<double, 7> g_ideal =
            this->tam_ideal_gravity->gravity(robot_state.q);
        for (int j = 0; j < 7; ++j) {
          tau_d_array[j] += g_ideal[j] - gravity_array[j];
        }
      }
      if (this->m_cfg.tam_enabled) {
        Eigen::VectorXd::Map(&tau_d_array[0], 7) +=
            tam_forward(tau_d_array, robot_state, gravity_array);
      }

      // tau after adding the TAM residual, before rate-limit / safety clip.
      const std::array<double, 7> tau_post_tam_array = tau_d_array;

      std::array<double, 7> tau_d_rate_limited =
          this->m_cfg.rate_limit
              ? franka::limitRate(franka::kMaxTorqueRate, tau_d_array,
                                  robot_state.tau_J_d)
              : tau_d_array;

      TorqueSafetyGuardFn(tau_d_rate_limited, torque_limit);

      if (this->m_cfg.tam_enabled) {
        // safe q, q_dot and tau
        const TAMHistorySample sample{.t = this->tam_now(),
                                      .q = robot_state.q,
                                      .dq = robot_state.dq,
                                      .tau_cmd = this->m_cfg.tam_history_pre_ratelimit
                                                     ? tau_post_tam_array
                                                     : tau_d_rate_limited,
                                      .gravity = gravity_array};
        this->tam_history.push_back(sample);
        this->tam_recent.push_back(sample);
        if (this->tam_recent.size() > 32) {
          this->tam_recent.pop_front();
        }
      }

      if (this->tam_logging.load()) {
        const std::chrono::high_resolution_clock::time_point t_end =
            std::chrono::high_resolution_clock::now();
        const TAMForwardDiag& d = this->last_tam_diag;
        TAMDebugSample dbg;
        dbg.t = this->tam_now();
        dbg.period = period.toSec();
        dbg.compute_us =
            static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                    t_end - t1)
                                    .count());
        dbg.q = robot_state.q;
        dbg.dq = robot_state.dq;
        dbg.tau_pd = tau_pd_array;
        dbg.tau_post_tam = tau_post_tam_array;
        dbg.tau_final = tau_d_rate_limited;
        dbg.gravity = gravity_array;
        dbg.tau_meas = robot_state.tau_J;
        for (int j = 0; j < 7; ++j) {
          dbg.desired_q[j] = desired_q[j];
          dbg.delta_raw[j] = d.delta_raw[j];
          dbg.delta_clipped[j] = d.delta_clipped[j];
          dbg.delta_applied[j] = d.delta_applied[j];
        }
        dbg.latent_norm = d.latent_norm;
        dbg.latent_age = d.latent_age;
        dbg.latent_size = d.latent_size;
        dbg.ramp = d.ramp;
        dbg.active_ticks = d.active_ticks;
        dbg.window_dt = d.window_dt;
        dbg.history_steps = d.history_steps;
        dbg.max_ood_tau = d.max_ood_tau;
        dbg.flags = (d.model_present ? 1 : 0) | (d.window_ok ? 2 : 0) |
                    (d.active ? 4 : 0);
        this->tam_debug_record(dbg);
      }

      return tau_d_rate_limited;
    });
  } catch (...) {
    this->background_exception.store(std::current_exception());
  }

  this->running_controller.store(Controller::none);
}

void Franka::zero_torque_guiding() {
  this->check_for_background_errors();
  if (this->running_controller.load() != Controller::none) {
    throw std::runtime_error(
        "A controller is currently running. Please stop it first.");
  }
  this->controller_time = 0.0;
  this->running_controller.store(Controller::ztc);
  this->control_thread = std::thread(&Franka::zero_torque_controller, this);
}

void Franka::zero_torque_controller() {
  adaptor::TryElevateControlThreadPriority(this->m_cfg.rt_priority);
  this->set_default_robot_behavior();
  if (this->m_cfg.allow_high_collision) {
    // High collision threshold values for high impedance.
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  }

  this->controller_time = 0.0;
  try {
    this->robot.control([&](const franka::RobotState& robot_state,
                            franka::Duration period) -> franka::Torques {
      this->curr_state.store(robot_state);

      this->interpolator_mutex.lock();
      this->controller_time += period.toSec();
      this->interpolator_mutex.unlock();
      if (this->running_controller.load() == Controller::none) {
        // stop
        return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
      }
      return franka::Torques({0, 0, 0, 0, 0, 0, 0});
    });
  } catch (...) {
    this->background_exception.store(std::current_exception());
  }

  this->running_controller.store(Controller::none);
}

void Franka::move_home() {
  // sync
  this->stop_control_thread();
  if (!this->m_cfg.q_home.has_value()) {
    std::cerr << "Home position is not defined in the config." << std::endl;
    return;
  }
  FrankaMotionGenerator motion_generator(this->m_cfg.speed_factor,
                                         this->m_cfg.q_home.value());
  this->robot.control(motion_generator);
}

void Franka::automatic_error_recovery() {
  this->robot.automaticErrorRecovery();
}

void Franka::reset() {
  this->stop_control_thread();
  this->automatic_error_recovery();
  this->clear_background_error();
}

void Franka::wait_milliseconds(int milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void Franka::double_tap_robot_to_continue() {
  auto s = this->robot.readOnce();
  int touch_counter = false;
  bool can_be_touched_again = true;
  Eigen::Vector3d start_force;
  auto last_time_something_happened = std::chrono::system_clock::now();
  auto last_time_something_touched = std::chrono::system_clock::now();
  start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
  this->robot.read([&](const franka::RobotState& robot_state) {
    Eigen::Vector3d force;
    force << robot_state.O_F_ext_hat_K[0], robot_state.O_F_ext_hat_K[1],
        robot_state.O_F_ext_hat_K[2];
    if ((start_force - force).norm() > 2 and can_be_touched_again) {
      touch_counter++;
      can_be_touched_again = false;
      last_time_something_happened = std::chrono::system_clock::now();
      last_time_something_touched = std::chrono::system_clock::now();
    }
    if (touch_counter > 0 and (start_force - force).norm() < 1) {
      can_be_touched_again = true;
      last_time_something_happened = std::chrono::system_clock::now();
    }
    std::chrono::duration<double> elapse_time_touched =
        std::chrono::system_clock::now() - last_time_something_touched;
    if (elapse_time_touched.count() > 0.5) {
      touch_counter = 0;
    }
    std::chrono::duration<double> elapse_time_happend =
        std::chrono::system_clock::now() - last_time_something_happened;
    if (elapse_time_happend.count() > 2) {
      start_force = force;
      last_time_something_happened = std::chrono::system_clock::now();
    }
    return touch_counter < 2;
  });
  wait_milliseconds(100);
}

double quintic_polynomial_speed_profile(double time, double start_time,
                                        double end_time) {
  double ret;
  if (time >= end_time) {
    time = end_time;
  }
  double deltaT = end_time - start_time;

  double tau = (time - start_time) / (deltaT);
  ret = (6.0 * std::pow(tau, 5.0) - 15.0 * std::pow(tau, 4.0) +
         10.0 * std::pow(tau, 3.0));
  return ret;
  // alternatively we can use a cosine profile:
  // return (1 - std::cos(M_PI * progress)) / 2.0;
}

std::optional<std::shared_ptr<common::Kinematics>> Franka::get_ik() {
  return this->m_ik;
}

void Franka::set_cartesian_position(const common::Pose& x) {
  // pose is assumed to be in the robots coordinate frame
  if (this->m_cfg.async_control) {
    this->osc_set_cartesian_position(x);
    return;
  }
  if (this->m_cfg.ik_solver == IKSolver::franka_ik) {
    const franka::RobotState robot_state = this->robot.readOnce();
    const common::Pose target_pose =
        x * this->m_cfg.tcp_offset.inverse() * common::Pose(robot_state.F_T_EE);
    this->set_cartesian_position_internal(target_pose, 1.0, std::nullopt,
                                          std::nullopt);

  } else if (this->m_cfg.ik_solver == IKSolver::rcs_ik) {
    this->set_cartesian_position_ik(x);
  }
}

void Franka::set_cartesian_position_ik(const common::Pose& pose) {
  if (!this->m_ik.has_value()) {
    throw std::runtime_error(
        "No inverse kinematics was provided. Cannot use IK to set cartesian "
        "position.");
  }
  auto joints = this->m_ik.value()->inverse(pose, this->get_joint_position(),
                                            this->m_cfg.tcp_offset);

  if (joints.has_value()) {
    this->set_joint_position(joints.value());
  } else {
    // throw error
    // throw std::runtime_error("IK failed");
    std::cerr << "IK failed";
  }
}

common::Pose Franka::get_base_pose_in_world_coordinates() {
  return this->m_cfg.world_to_robot.has_value()
             ? this->m_cfg.world_to_robot.value()
             : common::Pose();
}

void Franka::set_cartesian_position_internal(const common::Pose& pose,
                                             double max_time,
                                             std::optional<double> elbow,
                                             std::optional<double> max_force) {
  // TODO: use speed factor instead of max_time
  common::Pose initial_pose = this->get_cartesian_position();

  auto force_stop_condition = [&max_force](const franka::RobotState& state,
                                           const double progress) {
    Eigen::Vector3d force;
    force << state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1],
        state.O_F_ext_hat_K[2];
    double minimum_progress = 0.1;
    return force.norm() > max_force.value() and progress > minimum_progress;
  };

  std::array<double, 2> initial_elbow;
  double time = 0.0;
  bool should_stop = false;

  this->robot.control(
      [&force_stop_condition, &initial_elbow, &elbow, &max_force, &time,
       &max_time, &initial_pose, &should_stop, this,
       &pose](const franka::RobotState& state,
              franka::Duration time_step) -> franka::CartesianPose {
        time += time_step.toSec();
        if (time == 0) {
          initial_elbow = state.elbow_c;

          initial_pose = common::Pose(state.O_T_EE);
        }
        auto new_elbow = initial_elbow;
        const double progress = time / max_time;

        // calculate new pose by interpolating along the linear path
        common::Pose new_pose = initial_pose.interpolate(
            pose, quintic_polynomial_speed_profile(progress, 0, 1));

        if (elbow.has_value()) {
          new_elbow[0] += (elbow.value() - initial_elbow[0]) *
                          (1 - std::cos(M_PI * progress)) / 2.0;
        }
        if (max_force.has_value()) {
          should_stop = force_stop_condition(state, progress);
        }
        if (time >= max_time or should_stop) {
          if (elbow.has_value()) {
            return franka::MotionFinished({new_pose.affine_array(), new_elbow});
          }
          return franka::MotionFinished(new_pose.affine_array());
        }
        if (elbow.has_value()) {
          return {new_pose.affine_array(), new_elbow};
        }
        return new_pose.affine_array();
      },
      franka::ControllerMode::kCartesianImpedance);
}

}  // namespace hw
}  // namespace rcs
