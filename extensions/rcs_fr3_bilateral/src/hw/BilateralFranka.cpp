#include "BilateralFranka.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace rcs {
namespace hw {

namespace {
common::Vector7d robot_state_array_to_vector(
    const std::array<double, 7>& values) {
  common::Vector7d vector;
  vector = Eigen::Map<const common::Vector7d>(values.data());
  return vector;
}

int signum(double value) { return (0.0 < value) - (value < 0.0); }
}  // namespace

BilateralFranka::BilateralFranka(
    const BilateralFrankaConfig& cfg,
    std::optional<std::shared_ptr<common::Kinematics>> leader_ik,
    std::optional<std::shared_ptr<common::Kinematics>> follower_ik)
    : m_cfg(cfg),
      leader(std::make_shared<Franka>(cfg.leader_cfg, leader_ik)),
      follower(std::make_shared<Franka>(cfg.follower_cfg, follower_ik)) {}

BilateralFranka::~BilateralFranka() {
  try {
    stop();
  } catch (...) {
  }
}

void BilateralFranka::start() {
  check_for_background_errors();
  if (running.exchange(true)) {
    return;
  }

  try {
    capture_reference();
    if (gravity_only_mode()) {
      leader->zero_torque_guiding();
      follower->zero_torque_guiding();
    } else if (m_cfg.leader_haptic_feedback) {
      leader->controller_set_joint_torque(common::Vector7d::Zero());
    } else {
      leader->zero_torque_guiding();
    }
    leader_thread = std::thread(&BilateralFranka::leader_loop, this);
    follower_thread = std::thread(&BilateralFranka::follower_loop, this);
  } catch (...) {
    running.store(false);
    throw;
  }
}

void BilateralFranka::stop() {
  running.store(false);
  if (leader_thread.has_value() && leader_thread->joinable()) {
    leader_thread->join();
  }
  if (follower_thread.has_value() && follower_thread->joinable()) {
    follower_thread->join();
  }
  leader_thread.reset();
  follower_thread.reset();

  follower->stop_control_thread();
  leader->stop_control_thread();

  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state.running = false;
  }
  check_for_background_errors();
}

void BilateralFranka::move_home() {
  check_for_background_errors();
  stop();

  std::exception_ptr leader_exception = nullptr;
  std::exception_ptr follower_exception = nullptr;

  std::thread leader_home([&]() {
    try {
      leader->move_home();
    } catch (...) {
      leader_exception = std::current_exception();
    }
  });
  std::thread follower_home([&]() {
    try {
      follower->move_home();
    } catch (...) {
      follower_exception = std::current_exception();
    }
  });

  leader_home.join();
  follower_home.join();

  if (leader_exception) {
    std::rethrow_exception(leader_exception);
  }
  if (follower_exception) {
    std::rethrow_exception(follower_exception);
  }

  capture_reference();
}

void BilateralFranka::reset() {
  stop();
  leader->reset();
  follower->reset();
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state = BilateralFrankaState();
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex);
    has_leader_sample = false;
    latest_leader_q.setZero();
    latest_follower_target_q.setZero();
  }
}

bool BilateralFranka::is_running() const { return running.load(); }

BilateralFrankaConfig BilateralFranka::get_config() const { return m_cfg; }

BilateralFrankaState BilateralFranka::get_state() {
  check_for_background_errors();
  std::lock_guard<std::mutex> lock(state_mutex);
  state.running = running.load();
  return state;
}

std::shared_ptr<Franka> BilateralFranka::get_leader() { return leader; }

std::shared_ptr<Franka> BilateralFranka::get_follower() { return follower; }

void BilateralFranka::update_once() {
  check_for_background_errors();
  std::unique_ptr<FrankaState> leader_state(leader->get_state());
  std::unique_ptr<FrankaState> follower_state(follower->get_state());

  const common::Vector7d leader_q =
      robot_state_array_to_vector(leader_state->robot_state.q);
  const common::Vector7d leader_dq =
      robot_state_array_to_vector(leader_state->robot_state.dq);
  const common::Vector7d follower_q =
      robot_state_array_to_vector(follower_state->robot_state.q);
  const common::Vector7d follower_dq =
      robot_state_array_to_vector(follower_state->robot_state.dq);
  const common::Vector7d follower_external_tau = robot_state_array_to_vector(
      follower_state->robot_state.tau_ext_hat_filtered);

  bool has_reference = false;
  common::Vector7d previous_target = follower_q;
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    has_reference = state.has_reference;
    previous_target = state.follower_target_q;
  }

  if (!has_reference) {
    initial_leader_q = leader_q;
    initial_follower_q = follower_q;
  }

  const common::Vector7d raw_target = compute_follower_target(leader_q);
  const common::Vector7d target = limit_joint_step(
      has_reference ? previous_target : follower_q, raw_target);
  const common::Vector7d leader_torque_command =
      gravity_only_mode()
          ? common::Vector7d::Zero()
          : compute_leader_torque_command(follower_external_tau, leader_dq);

  if (!gravity_only_mode() && m_cfg.leader_haptic_feedback) {
    leader->controller_set_joint_torque(leader_torque_command);
  }
  if (!gravity_only_mode()) {
    follower->controller_set_joint_position(target);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state.leader_q = leader_q;
    state.leader_dq = leader_dq;
    state.follower_q = follower_q;
    state.follower_dq = follower_dq;
    state.follower_target_q = target;
    state.follower_external_tau = follower_external_tau;
    state.leader_torque_command = leader_torque_command;
    state.running = running.load();
    state.has_reference = true;
  }
}

void BilateralFranka::leader_loop() {
  try {
    while (running.load()) {
      std::unique_ptr<FrankaState> current_state(leader->get_state());
      const common::Vector7d leader_q =
          robot_state_array_to_vector(current_state->robot_state.q);
      const common::Vector7d leader_dq =
          robot_state_array_to_vector(current_state->robot_state.dq);

      common::Vector7d follower_external_tau;
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        follower_external_tau = state.follower_external_tau;
      }
      const common::Vector7d leader_torque_command =
          gravity_only_mode()
              ? common::Vector7d::Zero()
              : compute_leader_torque_command(follower_external_tau, leader_dq);
      if (!gravity_only_mode() && m_cfg.leader_haptic_feedback) {
        leader->controller_set_joint_torque(leader_torque_command);
      }

      {
        std::lock_guard<std::mutex> lock(command_mutex);
        latest_leader_q = leader_q;
        latest_follower_target_q = compute_follower_target(leader_q);
        has_leader_sample = true;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        state.leader_q = leader_q;
        state.leader_dq = leader_dq;
        state.leader_torque_command = leader_torque_command;
        state.running = true;
      }

      sleep_for_cycle();
    }
  } catch (...) {
    store_background_error();
    running.store(false);
  }
}

void BilateralFranka::follower_loop() {
  try {
    while (running.load()) {
      common::Vector7d target;
      common::Vector7d previous_target;
      bool should_command = false;
      {
        std::lock_guard<std::mutex> lock(command_mutex);
        if (has_leader_sample) {
          target = latest_follower_target_q;
          should_command = true;
        }
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex);
        previous_target = state.follower_target_q;
      }

      std::unique_ptr<FrankaState> follower_state(follower->get_state());
      const common::Vector7d follower_q =
          robot_state_array_to_vector(follower_state->robot_state.q);
      const common::Vector7d follower_dq =
          robot_state_array_to_vector(follower_state->robot_state.dq);
      const common::Vector7d follower_external_tau =
          robot_state_array_to_vector(
              follower_state->robot_state.tau_ext_hat_filtered);

      if (should_command && !gravity_only_mode()) {
        target = limit_joint_step(previous_target, target);
        follower->controller_set_joint_position(target);
      } else if (should_command) {
        target = limit_joint_step(previous_target, target);
      } else {
        target = follower_q;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex);
        state.follower_q = follower_q;
        state.follower_dq = follower_dq;
        state.follower_external_tau = follower_external_tau;
        state.follower_target_q = target;
        state.running = true;
      }

      sleep_for_cycle();
    }
  } catch (...) {
    store_background_error();
    running.store(false);
  }
}

bool BilateralFranka::gravity_only_mode() const {
  return m_cfg.control_mode == BilateralControlMode::gravity_only;
}

void BilateralFranka::capture_reference() {
  std::unique_ptr<FrankaState> leader_state(leader->get_state());
  std::unique_ptr<FrankaState> follower_state(follower->get_state());
  initial_leader_q = robot_state_array_to_vector(leader_state->robot_state.q);
  initial_follower_q =
      robot_state_array_to_vector(follower_state->robot_state.q);

  {
    std::lock_guard<std::mutex> lock(command_mutex);
    latest_leader_q = initial_leader_q;
    latest_follower_target_q = initial_follower_q;
    has_leader_sample = true;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    state.leader_q = initial_leader_q;
    state.follower_q = initial_follower_q;
    state.follower_target_q = initial_follower_q;
    state.running = running.load();
    state.has_reference = true;
  }
}

void BilateralFranka::check_for_background_errors() {
  std::lock_guard<std::mutex> lock(exception_mutex);
  if (background_exception) {
    std::exception_ptr ex = background_exception;
    background_exception = nullptr;
    std::rethrow_exception(ex);
  }
}

void BilateralFranka::store_background_error() {
  std::lock_guard<std::mutex> lock(exception_mutex);
  if (!background_exception) {
    background_exception = std::current_exception();
  }
}

void BilateralFranka::sleep_for_cycle() const {
  const double safe_rate = std::max(m_cfg.update_rate_hz, 1.0);
  const auto cycle = std::chrono::duration<double>(1.0 / safe_rate);
  std::this_thread::sleep_for(cycle);
}

common::Vector7d BilateralFranka::compute_follower_target(
    const common::Vector7d& leader_q) const {
  if (m_cfg.relative_joint_mapping) {
    return initial_follower_q +
           m_cfg.follower_joint_position_scale * (leader_q - initial_leader_q);
  }
  return m_cfg.follower_joint_position_scale * leader_q;
}

common::Vector7d BilateralFranka::compute_leader_torque_command(
    const common::Vector7d& follower_external_tau,
    const common::Vector7d& leader_dq) const {
  common::Vector7d command;
  for (int i = 0; i < command.size(); ++i) {
    const double tau_in = m_cfg.haptic_feedback_gain * follower_external_tau[i];
    const double power_in = leader_dq[i] * tau_in;
    double feedback_avoidance_term = 0.0;
    if (power_in < 0.0) {
      feedback_avoidance_term = -m_cfg.feedback_avoidance_alpha[i] *
                                signum(leader_dq[i]) * std::abs(power_in);
    }
    command[i] = -tau_in + feedback_avoidance_term;
  }
  return command;
}

common::Vector7d BilateralFranka::limit_joint_step(
    const common::Vector7d& current, const common::Vector7d& target) const {
  if (m_cfg.max_follower_joint_step <= 0.0) {
    return target;
  }
  common::Vector7d limited = target;
  for (int i = 0; i < limited.size(); ++i) {
    const double delta = target[i] - current[i];
    limited[i] = current[i] + std::clamp(delta, -m_cfg.max_follower_joint_step,
                                         m_cfg.max_follower_joint_step);
  }
  return limited;
}

}  // namespace hw
}  // namespace rcs
