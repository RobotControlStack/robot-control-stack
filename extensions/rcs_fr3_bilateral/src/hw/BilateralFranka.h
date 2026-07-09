#ifndef RCS_BILATERAL_FRANKA_H
#define RCS_BILATERAL_FRANKA_H

#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "hw/Franka.h"
#include "rcs/Kinematics.h"
#include "rcs/utils.h"

namespace rcs {
namespace hw {

enum class BilateralControlMode { bilateral = 0, gravity_only };

struct BilateralFrankaConfig {
  FrankaConfig leader_cfg;
  FrankaConfig follower_cfg;
  BilateralControlMode control_mode = BilateralControlMode::bilateral;
  double update_rate_hz = 1000.0;
  double follower_joint_position_scale = 1.0;
  double haptic_feedback_gain = 1.0;
  double max_follower_joint_step = 0.05;
  bool relative_joint_mapping = true;
  bool leader_haptic_feedback = true;
  common::Vector7d feedback_avoidance_alpha =
      (common::Vector7d() << 18.75, 15.0, 13.5, 9.0, 5.25, 3.0, 1.5).finished();
};

struct BilateralFrankaState {
  common::Vector7d leader_q = common::Vector7d::Zero();
  common::Vector7d leader_dq = common::Vector7d::Zero();
  common::Vector7d follower_q = common::Vector7d::Zero();
  common::Vector7d follower_dq = common::Vector7d::Zero();
  common::Vector7d follower_target_q = common::Vector7d::Zero();
  common::Vector7d follower_external_tau = common::Vector7d::Zero();
  common::Vector7d leader_torque_command = common::Vector7d::Zero();
  bool running = false;
  bool has_reference = false;
};

class BilateralFranka {
 public:
  BilateralFranka(const BilateralFrankaConfig& cfg,
                  std::optional<std::shared_ptr<common::Kinematics>> leader_ik =
                      std::nullopt,
                  std::optional<std::shared_ptr<common::Kinematics>>
                      follower_ik = std::nullopt);
  ~BilateralFranka();

  void start();
  void stop();
  void move_home();
  void update_once();
  void reset();

  bool is_running() const;
  BilateralFrankaConfig get_config() const;
  BilateralFrankaState get_state();
  std::shared_ptr<Franka> get_leader();
  std::shared_ptr<Franka> get_follower();

 private:
  BilateralFrankaConfig m_cfg;
  std::shared_ptr<Franka> leader;
  std::shared_ptr<Franka> follower;

  std::atomic<bool> running{false};
  std::optional<std::thread> leader_thread = std::nullopt;
  std::optional<std::thread> follower_thread = std::nullopt;

  mutable std::mutex state_mutex;
  mutable std::mutex command_mutex;
  mutable std::mutex exception_mutex;
  std::exception_ptr background_exception = nullptr;

  BilateralFrankaState state;
  common::Vector7d initial_leader_q = common::Vector7d::Zero();
  common::Vector7d initial_follower_q = common::Vector7d::Zero();
  common::Vector7d latest_leader_q = common::Vector7d::Zero();
  common::Vector7d latest_follower_target_q = common::Vector7d::Zero();
  bool has_leader_sample = false;

  void leader_loop();
  void follower_loop();
  bool gravity_only_mode() const;
  void capture_reference();
  void check_for_background_errors();
  void store_background_error();
  void sleep_for_cycle() const;

  common::Vector7d compute_follower_target(
      const common::Vector7d& leader_q) const;
  common::Vector7d compute_leader_torque_command(
      const common::Vector7d& follower_external_tau,
      const common::Vector7d& leader_dq) const;
  common::Vector7d limit_joint_step(const common::Vector7d& current,
                                    const common::Vector7d& target) const;
};

}  // namespace hw
}  // namespace rcs

#endif  // RCS_BILATERAL_FRANKA_H
