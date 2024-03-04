#include "FR3.h"

#include <franka/robot.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Exception.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/UrdfFactory.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "MotionGenerator.h"

namespace rcs {
namespace hw {
FR3::FR3(const std::string &ip, std::optional<const std::string &> filename)
    : robot(ip), model() {
  // set collision behavior and impedance
  this->set_default_robot_behavior();
  this->set_guiding_mode(true);

  if (filename.has_value()) {
    rl::mdl::UrdfFactory factory;
    factory.load(filename.value(), &model);
    ik = std::make_unique<rl::mdl::JacobianInverseKinematics>(
        static_cast<rl::mdl::Kinematic *>(&model));
  } else {
    ik = std::nullopt;
  }
}

FR3::~FR3() {}

void FR3::set_parameters(std::optional<double> speed_factor,
                         std::optional<const FR3Load &> load) {
  if (speed_factor.has_value()) {
    // make sure that the speed factor is between 0 and 1
    this->speed_factor = std::min(std::max(speed_factor.value(), 0.0), 1.0);
  }
  if (load.has_value()) {
    auto load_value = load.value();
    if (!load_value.f_x_cload.has_value()) {
      load_value.f_x_cload = Eigen::Vector3d::Zero();
    }
    if (!load_value.load_inertia.has_value()) {
      load_value.load_inertia = Eigen::Matrix3d::Zero();
    }

    this->robot.setLoad(
        load_value.load_mass,
        common::eigen2array<3, 1>(load_value.f_x_cload.value()),
        common::eigen2array<3, 3>(load_value.load_inertia.value()));
  }
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
  franka::RobotState state = this->robot.readOnce();
  common::Pose x(state.O_T_EE_c);
  return x;
}

void FR3::set_joint_position(const Vector7d &q) {
  // TODO: max force?
  MotionGenerator motion_generator(speed_factor, q);
  this->robot.control(motion_generator);
}

Vector7d FR3::get_joint_position() {
  franka::RobotState state = this->robot.readOnce();
  Vector7d joints(state.q.data());
  return joints;
}

void FR3::set_guiding_mode(bool enabled) {
  std::array<bool, 6> activated;
  activated.fill(enabled);
  this->robot.setGuidingMode(activated, enabled);
  this->guiding_mode_enabled = enabled;
}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::automatic_error_recovery() { this->robot.automaticErrorRecovery(); }

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

Eigen::Affine3d interpolate(const Eigen::Affine3d &start_pose,
                            const Eigen::Affine3d &dest_pose, double progress) {
  if (progress > 1) {
    progress = 1;
  }
  Eigen::Vector3d pos_result =
      start_pose.translation() +
      (dest_pose.translation() - start_pose.translation()) * progress;
  Eigen::Quaterniond quat_start, quat_end, quat_result;
  quat_start = start_pose.rotation();
  quat_end = dest_pose.rotation();
  quat_result = quat_start.slerp(progress, quat_end);
  Eigen::Affine3d result_pose;
  result_pose.setIdentity();
  result_pose.translation() = pos_result;
  result_pose.rotate(quat_result);
  return result_pose;
}

// todo this should be done with library function
std::array<double, 16> to_matrix(const Eigen::Affine3d &transform) {
  Eigen::Matrix3d rot_mat = transform.rotation();
  std::array<double, 16> mat = {rot_mat(0, 0),
                                rot_mat(1, 0),
                                rot_mat(2, 0),
                                0,
                                rot_mat(0, 1),
                                rot_mat(1, 1),
                                rot_mat(2, 1),
                                0,
                                rot_mat(0, 2),
                                rot_mat(1, 2),
                                rot_mat(2, 2),
                                0,
                                transform.translation().x(),
                                transform.translation().y(),
                                transform.translation().z(),
                                1};
  return mat;
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

// TODO: relative cartesian movement, cartesian movement with ik, cartesian with
// same orientation, cartesian with euler rotation

void FR3::set_cartesian_position(
    const common::Pose &x, IKController controller,
    const std::optional<common::Pose &> nominal_end_effector_frame) {
  common::Pose nominal_end_effector_frame_value;
  if (nominal_end_effector_frame.has_value()) {
    nominal_end_effector_frame_value = nominal_end_effector_frame.value();
  } else {
    // what does this exactly do, does it call the constructor?
    nominal_end_effector_frame_value = Eigen::Affine3d::Identity();
  }

  // TODO: trajectory planner
  if (controller == IKController::internal) {
    this->robot.setEE(nominal_end_effector_frame_value.affine_array());

    this->set_cartesian_position_internal(x, 1.0, std::nullopt, std::nullopt);
  } else if (controller == IKController::robotics_library) {
    this->set_cartesian_position_rl(x);
  }
}

void FR3::set_cartesian_position_rl(const common::Pose &x) {
  if (!this->ik.has_value()) {
    throw rl::mdl::Exception(
        "No file for robot model was provided. Cannot use RL for IK.");
  }
  this->ik.value()->addGoal(x.affine_matrix(), 0);
  bool success = this->ik.value()->solve();
  if (success) {
    rl::math::Vector q(6);
    q = this->model.getPosition();
    this->set_joint_position(q);
  } else {
    // throw error
    throw rl::mdl::Exception("IK failed");
  }
}

void FR3::set_cartesian_position_internal(const common::Pose &dest,
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
      [=, &initial_elbow, &time, &max_time, &initial_pose, &should_stop, &dest](
          const franka::RobotState &state,
          franka::Duration time_step) -> franka::CartesianPose {
        time += time_step.toSec();
        if (time == 0) {
          initial_elbow = state.elbow_c;

          // Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transMatrix(
          //     state.O_T_EE_c.data());
          // initial_pose.matrix() = transMatrix;
          initial_pose = common::Pose(state.O_T_EE_c);
        }
        auto new_elbow = initial_elbow;
        const double progress = time / max_time;

        // calculate new pose by interpolating along the linear path
        common::Pose new_pose = initial_pose.interpolate(
            dest, quintic_polynomial_speed_profile(progress, 0, 1));

        if (elbow.has_value()) {
          new_elbow[0] += (elbow.value() - initial_elbow[0]) *
                          (1 - std::cos(M_PI * progress)) / 2.0;
        }
        if (max_force.has_value()) {
          should_stop = force_stop_condition(state, progress);
        }
        should_stop = force_stop_condition(state, progress);
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