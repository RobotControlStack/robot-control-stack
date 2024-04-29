#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <mujoco/mujoco.h>

#include <Eigen/Eigen>
#include <list>
#include <memory>
#include <thread>

#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Kinematic.h"
#include "rl/mdl/Model.h"

namespace rcs {
namespace sim {
const common::Vector7d q_home((common::Vector7d() << 0, -M_PI_4, 0, -3 * M_PI_4,
                               0, M_PI_2, M_PI_4)
                                  .finished());
struct FR3Config : common::RConfig {
  size_t ik_duration = 300;  // milliseconds
  bool realtime;
  bool trajectory_trace;
};

struct FR3State : common::RState {
  bool ik_success = true;
  bool collision = false;
};

class FR3 : public common::Robot {
 public:
  FR3(mjModel *m, mjData *d, std::shared_ptr<rl::mdl::Model> mdl,
      std::optional<bool> render);
  FR3(const std::string &mjmdl, const std::string &rlmdl,
      std::optional<bool> render);
  ~FR3() override;
  bool set_parameters(const common::RConfig &cfg) override;
  std::unique_ptr<common::RConfig> get_parameters() override;
  std::unique_ptr<common::RState> get_state() override;
  common::Pose get_cartesian_position() override;
  void set_joint_position(const common::Vector7d &q) override;
  common::Vector7d get_joint_position() override;
  void move_home() override;
  void set_cartesian_position(const common::Pose &pose) override;
  void reset();
  std::list<mjvGeom>::iterator add_sphere(const common::Pose &pose, double size,
                                          const float *rgba);
  std::list<mjvGeom>::iterator add_line(const common::Pose &from,
                                        const common::Pose &to, double size,
                                        const float *rgba);
  void remove_marker(std::list<mjvGeom>::iterator &it);
  void clear_markers();

 private:
  FR3Config cfg;
  FR3State state;
  struct {
    struct {
      mjModel *mdl;
      mjData *data;
    } mj;
    struct {
      std::shared_ptr<rl::mdl::Model> mdl;
      std::shared_ptr<rl::mdl::Kinematic> kin;
      std::shared_ptr<rl::mdl::JacobianInverseKinematics> ik;
    } rl;
  } models;
  struct {
    std::set<size_t> arm;
    std::set<size_t> hand;
    std::set<size_t> gripper;
  } cgeom_ids;
  std::jthread render_thread;
  bool exit_requested;
  std::list<mjvGeom> markers;
  void wait_for_convergence(rcs::common::Vector7d target_angles);
  bool collision(std::set<size_t> const &geom_ids);
  void render_loop();
};
}  // namespace sim
}  // namespace rcs

#endif  // RCS_FR3SIM_H
