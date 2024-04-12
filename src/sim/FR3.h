#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <mujoco/mujoco.h>

#include <Eigen/Eigen>
#include <list>
#include <thread>

#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/Kinematic.h"
#include "rl/mdl/Model.h"

namespace rcs {
namespace sim {
const common::Vector7d q_home((common::Vector7d() << 0, -M_PI_4, 0, -3 * M_PI_4,
                               0, M_PI_2, M_PI_4)
                                  .finished());
const double DEFAULT_SPEED_FACTOR = 0.2;
struct FR3Load {
  double load_mass;
  std::optional<Eigen::Vector3d> f_x_cload;
  std::optional<Eigen::Matrix3d> load_inertia;
};

struct FR3Config : common::RConfig {
  double speed_factor;
  bool render;
  bool realtime;
  bool trajectory_trace;
  FR3Config(double speed_factor, bool render, bool realtime,
            bool trajectory_trace)
      : speed_factor(speed_factor),
        render(render),
        realtime(realtime),
        trajectory_trace(trajectory_trace){};
};

struct FR3State : common::RState {};

class FR3 : public common::Robot {
 public:
  FR3(const std::string &mjmdl, const std::string &rlmdl);
  ~FR3() override;
  bool set_parameters(const common::RConfig &cfg) override;
  std::unique_ptr<common::RConfig> get_parameters() override;
  std::unique_ptr<common::RState> get_state() override;
  common::Pose get_cartesian_position() override;
  void set_joint_position(const common::Vector7d &q) override;
  common::Vector7d get_joint_position() override;
  void move_home() override;
  void set_cartesian_position(const common::Pose &pose) override;
  std::list<mjvGeom>::iterator add_sphere(const common::Pose &pose, double size,
                                          const float *rgba);
  std::list<mjvGeom>::iterator add_line(const common::Pose &from,
                                        const common::Pose &to, double size,
                                        const float *rgba);
  void remove_marker(std::list<mjvGeom>::iterator &it);
  void clear_markers();

 private:
  FR3Config cfg;
  struct {
    struct {
      std::shared_ptr<mjModel> mdl;
      std::shared_ptr<mjData> data;
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
  void reset();
};
}  // namespace sim
}  // namespace rcs

#endif  // RCS_FR3SIM_H
