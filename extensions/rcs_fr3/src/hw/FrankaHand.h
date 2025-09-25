#ifndef RCS_FRANKA_HAND_H
#define RCS_FRANKA_HAND_H

#include <franka/gripper.h>

#include <Eigen/Core>
#include <cmath>
#include <string>
#include <thread>

#include "rcs/Robot.h"

// TODO: we need a common interface for the gripper, maybe we do use the hal
// we need to create a robot class that has both the robot and the gripper

// TODO: find out exact difference between grasp and move
// implement function to put the gripper to a specific position
// this might be done with a thread as the command is blocking

namespace rcs {
namespace hw {

struct FHConfig : common::GripperConfig {
  double grasping_width = 0.05;
  double speed = 0.1;
  double force = 5;
  double epsilon_inner = 0.005;
  double epsilon_outer = 0.005;
  bool async_control = false;
};

struct FHState : common::GripperState {
  double width;
  // true is open
  bool bool_state;
  bool is_grasped;
  uint16_t temperature;
  double last_commanded_width;
  double max_unnormalized_width;
  bool is_moving;
};

class FrankaHand : public common::Gripper {
 private:
  franka::Gripper gripper;
  FHConfig cfg;
  double max_width;
  double last_commanded_width;
  // TODO: might be better if is_moving is a lock
  bool is_moving = false;
  std::optional<std::thread> control_thread = std::nullopt;
  void m_reset();
  void m_stop();
  void m_wait();

 public:
  FrankaHand(const std::string &ip, const FHConfig &cfg);
  ~FrankaHand() override;

  bool set_config(const FHConfig &cfg);

  FHConfig *get_config() override;

  FHState *get_state() override;

  void reset() override;

  // normalized width of the gripper, 0 is closed, 1 is open
  void set_normalized_width(double width, double force = 0) override;
  double get_normalized_width() override;

  bool is_grasped() override;

  bool homing();

  void grasp() override;
  void open() override;
  void shut() override;
  void close() override {};
};
}  // namespace hw
}  // namespace rcs
#endif  // RCS_FRANKA_HAND_H