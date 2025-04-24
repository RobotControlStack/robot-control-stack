#include "FR3.h"

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
  this->robot.setCollisionBehavior({{2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0}},
                                   {{2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0}},
                                   {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                   {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                   {{2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0}},
                                   {{2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0}},
                                   {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                                   {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});
  this->robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  this->robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

common::Pose FR3::get_cartesian_position() {
  franka::RobotState state = this->robot.readOnce();
  common::Pose x(state.O_T_EE);
  return x;
}

void FR3::set_joint_position(const common::Vector7d &q) {
  // TODO: max force?
  MotionGenerator motion_generator(this->cfg.speed_factor, q);
  this->robot.control(motion_generator);
}

common::Vector7d FR3::get_joint_position() {
  franka::RobotState state = this->robot.readOnce();
  common::Vector7d joints(state.q.data());
  return joints;
}

void FR3::set_guiding_mode(bool enabled) {
  std::array<bool, 6> activated;
  activated.fill(enabled);
  this->robot.setGuidingMode(activated, enabled);
  this->cfg.guiding_mode_enabled = enabled;
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
