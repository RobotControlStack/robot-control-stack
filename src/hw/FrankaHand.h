#ifndef RCS_FRANKA_HAND_H
#define RCS_FRANKA_HAND_H

#include <common/Robot.h>
#include <franka/gripper.h>

#include <Eigen/Core>
#include <cmath>
#include <string>

// TODO: we need a common interface for the gripper, maybe we do use the hal
// we need to create a robot class that has both the robot and the gripper

// TODO: find out exact difference between grasp and move
// implement function to put the gripper to a specific position
// this might be done with a thread as the command is blocking

namespace rcs {
namespace hw {

struct FHConfig : common::GConfig {
  double grasping_width = 0.05;
  double speed = 0.1;
  double force = 5;
  double epsilon_inner = 0.005;
  double epsilon_outer = 0.005;
};

struct FHState : common::GState {
  double width;
  bool is_grasped;
  uint16_t temperature;
  double last_commanded_width;
  double max_unnormalized_width;
};

class FrankaHand : public common::Gripper {
 private:
  franka::Gripper gripper;
  FHConfig cfg;
  double max_width;
  double last_commanded_width;
  void m_reset();

 public:
  FrankaHand(const std::string &ip, const FHConfig &cfg);
  ~FrankaHand() override;

  bool set_parameters(const FHConfig &cfg);

  FHConfig *get_parameters() override;

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
};
}  // namespace hw
}  // namespace rcs
#endif  // RCS_FRANKA_HAND_H