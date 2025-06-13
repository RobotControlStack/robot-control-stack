#ifndef RCS_FRANKA_HAND_SIM_H
#define RCS_FRANKA_HAND_SIM_H

#include <common/Robot.h>

#include <Eigen/Core>
#include <cmath>
#include <set>
#include <string>

#include "sim/sim.h"

namespace rcs {
namespace sim {

struct SimGripperConfig : common::GripperConfig {
  double epsilon_inner = 0.005;
  double epsilon_outer = 0.005;
  double seconds_between_callbacks = 0.05;  // 20 Hz
  std::vector<std::string> ignored_collision_geoms = {};
  std::vector<std::string> collision_geoms{"hand_c", "d435i_collision",
                                           "finger_0_left", "finger_0_right"};

  std::vector<std::string> collision_geoms_fingers{"finger_0_left",
                                                   "finger_0_right"};
  std::string joint1 = "finger_joint1";
  std::string joint2 = "finger_joint2";
  std::string actuator = "actuator8";

  void add_id(const std::string &id) {
    for (auto &s : this->collision_geoms) {
      s = s + "_" + id;
    }
    for (auto &s : this->collision_geoms_fingers) {
      s = s + "_" + id;
    }
    for (auto &s : this->ignored_collision_geoms) {
      s = s + "_" + id;
    }
    this->joint1 = this->joint1 + "_" + id;
    this->joint2 = this->joint2 + "_" + id;
    this->actuator = this->actuator + "_" + id;
  }
};

struct SimGripperState : common::GripperState {
  double last_commanded_width = 0;
  double max_unnormalized_width = 0;
  bool is_moving = false;
  double last_width = 0;
  bool collision = false;
};

class SimGripper : public common::Gripper {
 private:
  const double MAX_WIDTH = 255;
  const double MAX_JOINT_WIDTH = 0.04;
  SimGripperConfig cfg;
  std::shared_ptr<Sim> sim;
  int actuator_id;
  int joint_id_1;
  int joint_id_2;
  SimGripperState state;
  bool convergence_callback();
  bool collision_callback();
  std::set<size_t> cgeom;
  std::set<size_t> cfgeom;
  std::set<size_t> ignored_collision_geoms;
  void add_collision_geoms(const std::vector<std::string> &cgeoms_str,
                           std::set<size_t> &cgeoms_set, bool clear_before);
  void m_reset();

 public:
  SimGripper(std::shared_ptr<Sim> sim, const SimGripperConfig &cfg);
  ~SimGripper() override;

  bool set_parameters(const SimGripperConfig &cfg);

  SimGripperConfig *get_parameters() override;

  SimGripperState *get_state() override;

  // normalized width of the gripper, 0 is closed, 1 is open
  void set_normalized_width(double width, double force = 0) override;
  double get_normalized_width() override;

  void reset() override;

  bool is_grasped() override;

  void grasp() override;
  void open() override;
  void shut() override;
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_FRANKA_HAND_SIM_H