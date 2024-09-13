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

struct FHConfig : common::GConfig {
  double epsilon_inner = 0.005;
  double epsilon_outer = 0.005;
  double seconds_between_callbacks = 0.05;  // 20 Hz
  std::vector<std::string> ignored_collision_geoms = {};
};

struct FHState : common::GState {
  double last_commanded_width = 0;
  double max_unnormalized_width = 0;
  bool is_moving = false;
  double last_width = 0;
  bool collision = false;
};

class FrankaHand : public common::Gripper {
 private:
  const double MAX_WIDTH = 255;
  const double MAX_JOINT_WIDTH = 0.04;
  FHConfig cfg;
  std::shared_ptr<Sim> sim;
  int actuator_id;
  int joint_id_1;
  int joint_id_2;
  FHState state;
  std::string id;
  void is_moving_callback();
  bool convergence_callback();
  bool collision_callback();
  std::set<size_t> cgeom;
  std::set<size_t> cfgeom;
  std::set<size_t> ignored_collision_geoms;
  void add_collision_geoms(const std::vector<std::string> &cgeoms_str,
                           std::set<size_t> &cgeoms_set, bool clear_before);
  void m_reset();

 public:
  FrankaHand(std::shared_ptr<Sim> sim, const std::string &id,
             const FHConfig &cfg);
  ~FrankaHand() override;

  bool set_parameters(const FHConfig &cfg);

  FHConfig *get_parameters() override;

  FHState *get_state() override;

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