#include "FR3.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "MotionGenerator.h"
#include "common/Pose.h"

namespace rcs {
namespace hw {
FR3::FR3(const std::string &ip, std::optional<std::shared_ptr<common::IK>> ik,
         const std::optional<FR3Config> &cfg)
    : robot(ip), m_ik(ik) {
  // set collision behavior and impedance
  this->set_default_robot_behavior();
  this->set_guiding_mode(true);

  if (cfg.has_value()) {
    this->cfg = cfg.value();
  }  // else default constructor
}

FR3::~FR3() {}

/**
 * @brief Set the parameters for the robot
 * @param cfg The configuration for the robot, it should be a FR3Config type
 * otherwise the call will fail
 */
bool FR3::set_parameters(const FR3Config &cfg) {
  this->cfg = cfg;
  this->cfg.speed_factor = std::min(std::max(cfg.speed_factor, 0.0), 1.0);

  if (this->cfg.load_parameters.has_value()) {
    auto load_value = &(this->cfg.load_parameters.value());
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

FR3Config *FR3::get_parameters() {
  // copy config to heap
  FR3Config *cfg = new FR3Config();
  *cfg = this->cfg;
  return cfg;
}

FR3State *FR3::get_state() {
  // dummy state until we define a prober state
  FR3State *state = new FR3State();
  return state;
}

void FR3::set_default_robot_behavior() {
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

common::Pose FR3::get_cartesian_position() {
  common::Pose x;
  if (!this->control_thread_running) {
    this->curr_state = this->robot.readOnce();
    x = common::Pose(this->curr_state.O_T_EE);
  } else {
    this->interpolator_mutex.lock();
    x = common::Pose(this->curr_state.O_T_EE);
    this->interpolator_mutex.unlock();
  }
  return x;
}

void FR3::set_joint_position(const common::Vector7d &q) {
  // TODO: max force?
  MotionGenerator motion_generator(this->cfg.speed_factor, q);
  this->robot.control(motion_generator);
}

common::Vector7d FR3::get_joint_position() {
  common::Vector7d joints;
  if (!this->control_thread_running) {
    this->curr_state = this->robot.readOnce();
    joints = common::Vector7d(this->curr_state.q.data());
  } else {
    this->interpolator_mutex.lock();
    joints = common::Vector7d(this->curr_state.q.data());
    this->interpolator_mutex.unlock();
  }
  return joints;
}

void FR3::set_guiding_mode(bool enabled) {
  std::array<bool, 6> activated;
  activated.fill(enabled);
  this->robot.setGuidingMode(activated, enabled);
  this->cfg.guiding_mode_enabled = enabled;
}

void PInverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_inv,
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

void TorqueSafetyGuardFn(std::array<double, 7> &tau_d_array, double min_torque,
                         double max_torque) {
  for (size_t i = 0; i < tau_d_array.size(); i++) {
    if (tau_d_array[i] < min_torque) {
      tau_d_array[i] = min_torque;
    } else if (tau_d_array[i] > max_torque) {
      tau_d_array[i] = max_torque;
    }
  }
}


void FR3::controller_set_joint_position(
    const common::Vector7d &desired_q) {

  // from deoxys/config/osc-position-controller.yml
  double traj_interpolation_time_fraction = 0.3; // in s
  // form deoxys/config/charmander.yml
  int policy_rate = 20;
  int traj_rate = 500;

  if (!this->control_thread_running) {
    this->controller_time = 0.0;
    this->get_joint_position();
  } else {
    this->interpolator_mutex.lock();
  }

  this->joint_interpolator.Reset(
      this->controller_time, Eigen::Map<common::Vector7d>(this->curr_state.q.data()), desired_q, policy_rate, traj_rate,
      traj_interpolation_time_fraction);

  // if not thread is running, then start
  if (!this->control_thread_running) {
    this->control_thread_running = true;
    this->control_thread = std::thread(&FR3::joint_controller, this);
  } else {
    this->interpolator_mutex.unlock();
  }
}

// todos
// - controller type
// - joint type
void FR3::osc_set_cartesian_position(
    const common::Pose &desired_pose_EE_in_base_frame) {
  // from deoxys/config/osc-position-controller.yml
  double traj_interpolation_time_fraction = 0.3;
  // form deoxys/config/charmander.yml
  int policy_rate = 20;
  int traj_rate = 500;

  if (!this->control_thread_running) {
    this->controller_time = 0.0;
    this->get_cartesian_position();
  } else {
    this->interpolator_mutex.lock();
  }

  common::Pose curr_pose(this->curr_state.O_T_EE);
  this->traj_interpolator.Reset(
      this->controller_time, curr_pose.translation(),
      curr_pose.quaternion(), desired_pose_EE_in_base_frame.translation(),
      desired_pose_EE_in_base_frame.quaternion(), policy_rate, traj_rate,
      traj_interpolation_time_fraction);

  // if not thread is running, then start
  if (!this->control_thread_running) {
    this->control_thread_running = true;
    this->control_thread = std::thread(&FR3::osc, this);
  } else {
    this->interpolator_mutex.unlock();
  }
}


// method to stop thread
void FR3::stop_control_thread() {
  if (this->control_thread.has_value() && this->control_thread_running) {
    this->control_thread_running = false;
    this->control_thread->join();
    this->control_thread.reset();
    // this->osc_desired_pos_EE_in_base_frame.reset();
  }
}

void FR3::osc() {
  franka::Model model = this->robot.loadModel();

  this->controller_time = 0.0;

  this->robot.setCollisionBehavior(
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  // from bench mark
  // ([150.0, 150.0, 60.0], 250.0), // kp_translation, kp_rotation
  // ([60.0, 150.0, 150.0], 250.0), // kd_translation, kd_rotation

  // from config file
  // Kp:
  // translation: [150.0, 150.0, 150.0]
  // rotation: 250.0

  Eigen::Matrix<double, 3, 3> Kp_p, Kp_r, Kd_p, Kd_r;
  Eigen::Matrix<double, 7, 1> static_q_task_;
  Eigen::Matrix<double, 7, 1> residual_mass_vec_;
  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;
  Eigen::Array<double, 7, 1> avoidance_weights_;

  // values from deoxys/config/osc-position-controller.yml
  Kp_p.diagonal() << 150, 150, 150;
  Kp_r.diagonal() << 250, 250, 250;

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

  this->robot.control([&](const franka::RobotState &robot_state,
                          franka::Duration period) -> franka::Torques {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    // torques handler
    if (!this->control_thread_running) {
      // TODO: test if this also works when the robot is moving
      franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      return franka::MotionFinished(zero_torques);
    }
    // TO BE replaced
    // if (!this->control_thread_running && dq.maxCoeff() < 0.0001) {
    //   return franka::MotionFinished(franka::Torques(tau_d_array));
    // }

    Eigen::Vector3d desired_pos_EE_in_base_frame;
    Eigen::Quaterniond desired_quat_EE_in_base_frame;

    common::Pose pose(robot_state.O_T_EE);
    // auto pose = this->get_cartesian_position();
    // from deoxys/config/osc-position-controller.yml
    double traj_interpolation_time_fraction = 0.3;
    // form deoxys/config/charmander.yml
    int policy_rate = 20;
    int traj_rate = 500;

    this->interpolator_mutex.lock();
    // if (this->controller_time == 0) {
    //   this->traj_interpolator.Reset(
    //       0., pose.translation(), pose.quaternion(),
    //       desired_pos_EE_in_base_frame, fixed_desired_quat_EE_in_base_frame,
    //       policy_rate, traj_rate, traj_interpolation_time_fraction);
    // }
    this->curr_state = robot_state;
    this->controller_time += period.toSec();
    this->traj_interpolator.GetNextStep(this->controller_time,
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
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

    std::array<double, 42> jacobian_array =
        model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
        jacobian_array.data());

    Eigen::MatrixXd jacobian_pos(3, 7);
    Eigen::MatrixXd jacobian_ori(3, 7);
    jacobian_pos << jacobian.block(0, 0, 3, 7);
    jacobian_ori << jacobian.block(3, 0, 3, 7);

    // End effector pose in base frame
    Eigen::Affine3d T_EE_in_base_frame(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
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

    pos_error =
        pos_error.unaryExpr([](double x) { return (abs(x) < 1e-4) ? 0. : x; });
    ori_error =
        ori_error.unaryExpr([](double x) { return (abs(x) < 5e-3) ? 0. : x; });

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
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

    std::array<double, 7> tau_d_rate_limited = franka::limitRate(
        franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);

    // deoxys/config/control_config.yml
    double min_torque = -5;
    double max_torque = 5;
    TorqueSafetyGuardFn(tau_d_rate_limited, min_torque, max_torque);

    return tau_d_rate_limited;
  });
}

void FR3::joint_controller() {
  franka::Model model = this->robot.loadModel();
  this->controller_time = 0.0;
  this->robot.setCollisionBehavior(
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


  // deoxys/config/joint-impedance-controller.yml
  common::Vector7d Kp;
  Kp << 100., 100., 100., 100., 75., 150., 50.;

  common::Vector7d Kd;
  Kd << 20., 20., 20., 20., 7.5, 15.0, 5.0;
 

  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;


  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  this->robot.control([&](const franka::RobotState &robot_state,
                          franka::Duration period) -> franka::Torques {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();


    // torques handler
    if (!this->control_thread_running) {
      // TODO: test if this also works when the robot is moving
      franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
      return franka::MotionFinished(zero_torques);
    }


    common::Vector7d desired_q;
    common::Pose pose(robot_state.O_T_EE);

    this->interpolator_mutex.lock();
    this->curr_state = robot_state;
    this->controller_time += period.toSec();
    this->joint_interpolator.GetNextStep(this->controller_time,
                                        desired_q);
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

    return tau_d_array;
  });
}

void FR3::zero_torque() {
  // start thread

  robot.setCollisionBehavior(
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
      {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

  double time = 0.0;
  this->robot.control([&](const franka::RobotState &state,
                          franka::Duration time_step) -> franka::Torques {
    time += time_step.toSec();
    if (time > 30) {
      // stop
      return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
    }
    return franka::Torques({0, 0, 0, 0, 0, 0, 0});
  });
}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::automatic_error_recovery() { this->robot.automaticErrorRecovery(); }

void FR3::reset() {
  this->automatic_error_recovery();
  this->move_home();
}

void FR3::wait_milliseconds(int milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void FR3::double_tap_robot_to_continue() {
  auto s = this->robot.readOnce();
  int touch_counter = false;
  bool can_be_touched_again = true;
  Eigen::Vector3d start_force;
  auto last_time_something_happened = std::chrono::system_clock::now();
  auto last_time_something_touched = std::chrono::system_clock::now();
  start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
  this->robot.read([&](const franka::RobotState &robot_state) {
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

std::optional<std::shared_ptr<common::IK>> FR3::get_ik() { return this->m_ik; }

void FR3::set_cartesian_position(const common::Pose &x) {
  // pose is assumed to be in the robots coordinate frame
  common::Pose nominal_end_effector_frame_value;
  if (this->cfg.nominal_end_effector_frame.has_value()) {
    nominal_end_effector_frame_value =
        this->cfg.nominal_end_effector_frame.value();
  } else {
    nominal_end_effector_frame_value = common::Pose::Identity();
  }

  if (this->cfg.controller == IKController::internal) {
    // if gripper is attached the tcp offset will automatically be applied
    // by libfranka
    this->robot.setEE(nominal_end_effector_frame_value.affine_array());
    this->set_cartesian_position_internal(x, 1.0, std::nullopt, std::nullopt);

  } else if (this->cfg.controller == IKController::robotics_library) {
    this->set_cartesian_position_ik(x);
  }
}

void FR3::set_cartesian_position_ik(const common::Pose &pose) {
  if (!this->m_ik.has_value()) {
    throw std::runtime_error(
        "No inverse kinematics was provided. Cannot use IK to set cartesian "
        "position.");
  }
  auto joints = this->m_ik.value()->ik(pose, this->get_joint_position(),
                                       this->cfg.tcp_offset);

  if (joints.has_value()) {
    this->set_joint_position(joints.value());
  } else {
    // throw error
    // throw std::runtime_error("IK failed");
    std::cerr << "IK failed";
  }
}

common::Pose FR3::get_base_pose_in_world_coordinates() {
  return this->cfg.world_to_robot.has_value() ? this->cfg.world_to_robot.value()
                                              : common::Pose();
}

void FR3::set_cartesian_position_internal(const common::Pose &pose,
                                          double max_time,
                                          std::optional<double> elbow,
                                          std::optional<double> max_force) {
  // TODO: use speed factor instead of max_time
  common::Pose initial_pose = this->get_cartesian_position();

  auto force_stop_condition = [&max_force](const franka::RobotState &state,
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
       &max_time, &initial_pose, &should_stop,
       &pose](const franka::RobotState &state,
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
