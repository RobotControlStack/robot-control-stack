#include "Franka.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "FrankaMotionGenerator.h"
#include "rcs/LinearInterpolator.h"
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

void Franka::controller_set_joint_position(const common::Vector7d& desired_q) {
  this->check_for_background_errors();
  const bool starting_fresh =
      this->running_controller.load() == Controller::none;

  if (starting_fresh) {
    this->controller_time = 0.0;
    this->get_joint_position();
    this->create_dynamics();
    common::FR3JointControllerConfig config;
    config.kp = this->m_cfg.kp;
    config.kd = this->m_cfg.kd;
    config.torque_limit = this->m_cfg.torque_limit;
    // a goal per policy step, interpolated over one policy period
    this->fr3_joint_controller = std::make_shared<common::FR3JointController>(
        config,
        std::make_shared<common::LinearJointInterpolator>(
            1.0 / this->m_cfg.policy_rate),
        this->m_cfg.joint_limits);
    this->fr3_joint_controller->reset(
        this->controller_state(this->curr_state.load(), 0.0));
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

  this->fr3_joint_controller->set_goal(
      desired_q,
      approach_on_start ? std::optional<double>(approach_time) : std::nullopt);

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
  const bool starting_fresh =
      this->running_controller.load() == Controller::none;

  if (starting_fresh) {
    this->controller_time = 0.0;
    this->curr_state.store(this->robot.readOnce());
    this->create_dynamics();
    common::FR3OSCConfig config;
    config.kp_position = this->m_cfg.kp_p;
    config.kp_orientation = this->m_cfg.kp_r;
    config.torque_limit = this->m_cfg.torque_limit;
    // a goal per policy step, interpolated over one policy period
    this->fr3_osc = std::make_shared<common::FR3OSC>(
        config,
        std::make_shared<common::LinearCartesianInterpolator>(
            1.0 / this->m_cfg.policy_rate),
        this->m_cfg.joint_limits);
    this->fr3_osc->reset(this->controller_state(this->curr_state.load(), 0.0));
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

  this->fr3_osc->set_goal(
      desired_pose_EE_in_base_frame,
      approach_on_start ? std::optional<double>(approach_time) : std::nullopt);

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

void Franka::create_dynamics() {
  if (!this->dynamics) {
    const std::string& path = this->m_cfg.kinematic_model_path;
    const bool urdf = path.size() >= 5 && path.ends_with(".urdf");
    this->dynamics = std::make_shared<common::PinocchioDynamics>(
        path, urdf, this->m_cfg.attachment_site);
  }
  this->dynamics->set_tcp_offset(this->m_cfg.tcp_offset);
}

common::ControllerState Franka::controller_state(
    const franka::RobotState& robot_state, double dt) {
  // the end effector and load configured in Desk, as franka::Model uses them
  common::Payload payload;
  payload.mass = robot_state.m_total;
  payload.com =
      Eigen::Map<const Eigen::Vector3d>(robot_state.F_x_Ctotal.data());
  payload.inertia =
      Eigen::Map<const Eigen::Matrix3d>(robot_state.I_total.data());
  common::Payload current = this->dynamics->get_payload();
  if (payload.mass != current.mass || payload.com != current.com ||
      payload.inertia != current.inertia) {
    this->dynamics->set_payload(payload);
  }
  return this->dynamics->compute(
      Eigen::Map<const common::Vector7d>(robot_state.q.data()),
      Eigen::Map<const common::Vector7d>(robot_state.dq.data()), dt);
}

void Franka::osc() {
  const bool allow_high_collision = this->m_cfg.allow_high_collision;

  this->controller_time = 0.0;

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

  try {
    this->robot.control([&](const franka::RobotState& robot_state,
                            franka::Duration period) -> franka::Torques {
      if (this->running_controller.load() == Controller::none) {
        return franka::MotionFinished(franka::Torques(robot_state.tau_J_d));
      }
      this->curr_state.store(robot_state);

      std::lock_guard<std::mutex> lock(this->interpolator_mutex);
      this->controller_time += period.toSec();
      common::VectorXd tau = this->fr3_osc->step(
          this->controller_state(robot_state, period.toSec()));
      std::array<double, 7> tau_array{};
      common::VectorXd::Map(tau_array.data(), 7) = tau;
      return tau_array;
    });
  } catch (...) {
    this->background_exception.store(std::current_exception());
  }

  // Ensure we mark the controller as stopped so we can restart later
  this->running_controller.store(Controller::none);
}

void Franka::joint_controller() {
  const bool allow_high_collision = this->m_cfg.allow_high_collision;
  this->controller_time = 0.0;

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

  try {
    this->robot.control([&](const franka::RobotState& robot_state,
                            franka::Duration period) -> franka::Torques {
      if (this->running_controller.load() == Controller::none) {
        return franka::MotionFinished(franka::Torques(robot_state.tau_J_d));
      }
      this->curr_state.store(robot_state);

      std::lock_guard<std::mutex> lock(this->interpolator_mutex);
      this->controller_time += period.toSec();
      common::VectorXd tau = this->fr3_joint_controller->step(
          this->controller_state(robot_state, period.toSec()));
      std::array<double, 7> tau_array{};
      common::VectorXd::Map(tau_array.data(), 7) = tau;
      return tau_array;
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
