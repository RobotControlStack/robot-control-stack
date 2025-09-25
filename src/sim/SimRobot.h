#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <mujoco/mujoco.h>
#include <rcs/IK.h>
#include <rcs/Pose.h>
#include <rcs/Robot.h>
#include <rcs/utils.h>

#include "sim/sim.h"

namespace rcs {
namespace sim {

struct SimRobotConfig : common::RobotConfig {
  double joint_rotational_tolerance =
      .05 * (std::numbers::pi / 180.0);    // 0.05 degree
  double seconds_between_callbacks = 0.1;  // 10 Hz
  bool realtime = false;
  bool trajectory_trace = false;
  std::vector<std::string> arm_collision_geoms{
      "fr3_link0_collision", "fr3_link1_collision", "fr3_link2_collision",
      "fr3_link3_collision", "fr3_link4_collision", "fr3_link5_collision",
      "fr3_link6_collision", "fr3_link7_collision"};
  std::vector<std::string> joints = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
  std::vector<std::string> actuators = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
  std::string base = "base";
  std::string mjcf_scene_path = "assets/scenes/fr3_empty_world/scene.xml";

  void add_id(const std::string &id) {
    for (auto &s : this->arm_collision_geoms) {
      s = s + "_" + id;
    }
    for (auto &s : this->joints) {
      s = s + "_" + id;
    }
    for (auto &s : this->actuators) {
      s = s + "_" + id;
    }
    this->attachment_site = this->attachment_site + "_" + id;
    this->base = this->base + "_" + id;
  }
};

struct SimRobotState : common::RobotState {
  common::VectorXd previous_angles;
  common::VectorXd target_angles;
  common::Pose inverse_tcp_offset;
  bool ik_success = true;
  bool collision = false;
  bool is_moving = false;
  bool is_arrived = false;
};

class SimRobot : public common::Robot {
 public:
  SimRobot(std::shared_ptr<rcs::sim::Sim> sim, std::shared_ptr<common::IK> ik,
           SimRobotConfig cfg, bool register_convergence_callback = true);
  ~SimRobot() override;
  bool set_config(const SimRobotConfig &cfg);
  SimRobotConfig *get_config() override;
  SimRobotState *get_state() override;
  common::Pose get_cartesian_position() override;
  void set_joint_position(const common::VectorXd &q) override;
  common::VectorXd get_joint_position() override;
  void move_home() override;
  void set_cartesian_position(const common::Pose &pose) override;
  common::Pose get_base_pose_in_world_coordinates() override;
  std::optional<std::shared_ptr<common::IK>> get_ik() override;
  void reset() override;
  void set_joints_hard(const common::VectorXd &q);
  void close() override {};

 private:
  SimRobotConfig cfg;
  SimRobotState state;
  std::shared_ptr<Sim> sim;
  std::shared_ptr<common::IK> m_ik;
  struct {
    std::set<size_t> cgeom;
    int attachment_site;
    std::vector<int> joints;
    std::vector<int> actuators;
    int base;
  } ids;
  void is_moving_callback();
  void is_arrived_callback();
  bool collision_callback();
  bool convergence_callback();
  void init_ids();
  void construct();
  void m_reset();
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_FR3SIM_H
