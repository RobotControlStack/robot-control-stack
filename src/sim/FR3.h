#ifndef RCS_FR3SIM_H
#define RCS_FR3SIM_H
#include <common/Pose.h>
#include <common/Robot.h>
#include <mujoco/mujoco.h>
#include <common/utils.h>

#include <Eigen/Eigen>
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
  FR3Config(double speed_factor, bool render, bool realtime) : speed_factor(speed_factor), render(render), realtime(realtime) {};
};

struct FR3State : common::RState {};

class FR3 : public common::Robot {
 private:
  std::shared_ptr<mjModel> mjmdl;
  std::shared_ptr<mjData> mujoco_data;
  std::shared_ptr<rl::mdl::Model> rlmdl;
  FR3Config cfg;
  std::jthread render_thread;
  bool exit_requested;
  std::shared_ptr<rl::mdl::Kinematic> kinmdl;
  rl::mdl::JacobianInverseKinematics ikmdl;

 public:
  FR3(const std::string mjmdl, const std::string rlmdl);
  ~FR3() override;
  bool set_parameters(const common::RConfig &cfg) override;
  std::unique_ptr<common::RConfig> get_parameters() override;
  std::unique_ptr<common::RState> get_state() override;
  common::Pose get_cartesian_position() override;
  void set_joint_position(const common::Vector7d &q) override;
  common::Vector7d get_joint_position() override;
  void move_home() override;
  void set_cartesian_position(const common::Pose &pose) override;
  void render_loop();
};
}  // namespace sim
}  // namespace rcs

#endif  // RCS_FR3SIM_H
