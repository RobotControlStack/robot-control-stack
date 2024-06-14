#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <mujoco/mujoco.h>

#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Kinematic.h"
#include "rl/mdl/Model.h"
#include "sim/sim.h"

namespace rcs {
namespace sim {

const common::Vector7d q_home((common::Vector7d() << 0, -M_PI_4, 0, -3 * M_PI_4,
                               0, M_PI_2, M_PI_4)
                                  .finished());

struct FR3Config : common::RConfig {
  size_t ik_duration = 300;  // milliseconds
  bool realtime = false;
  bool trajectory_trace = false;
  rcs::common::Pose tcp_offset = rcs::common::Pose::Identity();
};

struct FR3State : common::RState {
  common::Vector7d previous_angles;
  common::Vector7d target_angles;
  common:: Pose inverse_tcp_offset;
  bool ik_success = true;
  bool collision = false;
  bool is_moving = false;
  bool is_arrived = false;
};

class FR3 : public common::Robot {
 public:
  FR3(std::shared_ptr<rcs::sim::Sim> sim, std::string &id, std::shared_ptr<rl::mdl::Model> rlmdl);
  ~FR3() override;
  bool set_parameters(const FR3Config &cfg);
  FR3Config *get_parameters() override;
  FR3State *get_state() override;
  common::Pose get_cartesian_position() override;
  void set_joint_position(const common::Vector7d &q) override;
  common::Vector7d get_joint_position() override;
  void move_home() override;
  void set_cartesian_position(const common::Pose &pose) override;
  void reset();

 private:
  FR3Config cfg;
  FR3State state;
  Sim sim;
  std::string id;
  struct {
    std::shared_ptr<rl::mdl::Model> mdl;
    std::shared_ptr<rl::mdl::Kinematic> kin;
    std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
  } rl;
  struct {
    struct {
      std::set<size_t> arm;
      std::set<size_t> hand;
      std::set<size_t> gripper;
    } cgeom;
    int attachment_site;
    std::array<int, 7> joints;
    std::array<int, 7> ctrl;
  } ids;
  void wait_for_convergence(rcs::common::Vector7d target_angles);
  bool collision(std::set<size_t> const &geom_ids);
  void init_ids();
};
}  // namespace sim
}  // namespace rcs
#endif  // RCS_FR3SIM_H
