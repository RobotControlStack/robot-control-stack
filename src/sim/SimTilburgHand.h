#ifndef RCS_FRANKA_HAND_SIM_H
#define RCS_FRANKA_HAND_SIM_H

#include <Eigen/Core>
#include <cmath>
#include <set>
#include <string>

#include "rcs/Robot.h"
#include "sim/sim.h"

namespace rcs {
namespace sim {

struct SimTilburgHandConfig : common::HandConfig {
  rcs::common::VectorXd max_joint_position = (rcs::common::VectorXd(16) <<
      1.66, 1.57, 1.75, 1.57, 1.66, 1.66, 1.66, 0.44,
      1.66, 1.66, 1.66, 0.44, 1.66, 1.66, 1.66, 0.44).finished();
  rcs::common::VectorXd min_joint_position = (rcs::common::VectorXd(16) <<
      -0.09, 0.00,  0.00, 0.00,  -0.09, -0.09, 0.00, -0.44,
      -0.09, -0.09, 0.00, -0.44, -0.09, -0.09, 0.00, -0.44).finished();
  std::vector<std::string> ignored_collision_geoms = {};
  std::vector<std::string> collision_geoms =
      {};  //{"hand_c", "d435i_collision",
           // "finger_0_left", "finger_0_right"};

  std::vector<std::string> collision_geoms_fingers = {};  //{"finger_0_left",
                                                          //"finger_0_right"};
  std::vector<std::string> joints = {
      "thumb_cmc",        "thumb_mcp_rot", "thumb_mcp",  "thumb_ip",
      "index_mcp_abadd",  "index_mcp",     "index_pip",  "index_dip",
      "middle_mcp_abadd", "middle_mcp",    "middle_pip", "middle_dip",
      "ring_mcp_abadd",   "ring_mcp",      "ring_pip",   "ring_dip"};
  std::vector<std::string> actuators = {
      "thumb_cmc",        "thumb_mcp_rot", "thumb_mcp",  "thumb_ip",
      "index_mcp_abadd",  "index_mcp",     "index_pip",  "index_dip",
      "middle_mcp_abadd", "middle_mcp",    "middle_pip", "middle_dip",
      "ring_mcp_abadd",   "ring_mcp",      "ring_pip",   "ring_dip"};
  
  common::GraspType grasp_type = common::GraspType::POWER_GRASP;

  double seconds_between_callbacks = 0.0167;  // 60 Hz
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
    for (auto &s : this->joints) {
      s = s + "_" + id;
    }
    for (auto &s : this->actuators) {
      s = s + "_" + id;
    }
  }
};

struct SimTilburgHandState : common::HandState {
  rcs::common::VectorXd last_commanded_qpos = rcs::common::VectorXd::Zero(16);
  bool is_moving = false;
  rcs::common::VectorXd last_width = rcs::common::VectorXd::Zero(16);
  bool collision = false;
};

class SimTilburgHand : public common::Hand {
 private:
  SimTilburgHandConfig cfg;
  std::shared_ptr<Sim> sim;
  const int n_joints = 16;
  std::vector<int> actuator_ids = std::vector<int>(n_joints);
  std::vector<int> joint_ids = std::vector<int>(n_joints);
  SimTilburgHandState state;
  bool convergence_callback();
  bool collision_callback();
  std::set<size_t> cgeom;
  std::set<size_t> cfgeom;
  std::set<size_t> ignored_collision_geoms;
  void add_collision_geoms(const std::vector<std::string> &cgeoms_str,
                           std::set<size_t> &cgeoms_set, bool clear_before);
  void m_reset();

  const rcs::common::VectorXd open_pose =
      (rcs::common::VectorXd(16) << 0.0, 0.0, 0.5, 1.4, 0.2, 0.2, 0.2, 0.7, 0.2,
       0.2, 0.2, 0.3, 0.2, 0.2, 0.2, 0.0)
          .finished();
  const rcs::common::VectorXd power_grasp_pose =
      (rcs::common::VectorXd(16) << 0.5, 0.5, 0.5, 1.4, 0.5, 0.5, 1.0, 0.7, 0.5,
       0.5, 1.0, 0.3, 0.5, 0.5, 1.0, 0.0)
          .finished();
  bool set_grasp_type(common::GraspType grasp_type);

 public:
  SimTilburgHand(std::shared_ptr<Sim> sim, const SimTilburgHandConfig &cfg);
  ~SimTilburgHand() override;

  bool set_parameters(const SimTilburgHandConfig &cfg);

  SimTilburgHandConfig *get_parameters() override;

  SimTilburgHandState *get_state() override;

  // normalized joints of the hand, 0 is closed, 1 is open
  void set_normalized_joint_poses(const rcs::common::VectorXd &q) override;
  rcs::common::VectorXd get_normalized_joint_poses() override;

  void reset() override;

  bool is_grasped() override;

  void grasp() override;
  void open() override;
  void shut() override;
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_FRANKA_HAND_SIM_H