#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <common/IK.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <mujoco/mujoco.h>

#include "sim/sim.h"

namespace rcs {
namespace sim {

struct SimRobotConfig : common::RobotConfig {
  rcs::common::Pose tcp_offset = rcs::common::Pose::Identity();
  double joint_rotational_tolerance =
      .05 * (std::numbers::pi / 180.0);    // 0.05 degree
  double seconds_between_callbacks = 0.1;  // 10 Hz
  bool realtime = false;
  bool trajectory_trace = false;
  common::RobotType robot_type = common::RobotType::FR3;
  std::vector<std::string> arm_collision_geoms{};
  std::vector<std::string> joints = {
      "1", "2", "3", "4", "5",
  };
  std::vector<std::string> actuators = {
      "1", "2", "3", "4", "5",
  };
  std::string attachment_site = "gripper";
  std::string base = "base";

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
  bool set_parameters(const SimRobotConfig &cfg);
  SimRobotConfig *get_parameters() override;
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

 private:
  SimRobotConfig cfg;
  SimRobotState state;
  std::shared_ptr<Sim> sim;
  std::shared_ptr<common::IK> m_ik;
  struct {
    std::set<size_t> cgeom;
    int attachment_site;
    std::array<int, 7> joints;
    std::array<int, 7> ctrl;
    std::array<int, 7> actuators;
    int base;
  } ids;
  void is_moving_callback();
  void is_arrived_callback();
  bool collision_callback();
  bool convergence_callback();
  void init_ids();
  void construct();
  void m_reset();
  common::VectorXd to_mjc(const common::VectorXd& q);
  common::VectorXd to_hf(const common::VectorXd& q);
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_FR3SIM_H
