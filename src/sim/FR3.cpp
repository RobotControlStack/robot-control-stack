#include "FR3.h"

#include <algorithm>
#include <cstring>
#include <memory>

#include "Robot.h"
#include "glfw_adapter.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"

namespace rcs {
namespace sim {
FR3::FR3(std::string const mjmdl, std::string const rlmdl) : ikmdl(NULL) {
  this->mjmdl =
      std::shared_ptr<mjModel>(mj_loadXML(mjmdl.c_str(), NULL, NULL, 0));
  this->rlmdl = rl::mdl::UrdfFactory().create(rlmdl);
  this->mujoco_data = std::shared_ptr<mjData>(mj_makeData(this->mjmdl.get()));
  this->render_thread = std::jthread(std::bind(&FR3::render_loop, this));
  this->exit_requested = false;
  this->kinmdl = std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rlmdl);
  this->ikmdl = rl::mdl::JacobianInverseKinematics(this->kinmdl.get());
  ikmdl.setDuration(std::chrono::milliseconds(100));
}

bool FR3::set_parameters(common::RConfig const& cfg) {
  this->cfg = dynamic_cast<FR3Config const&>(cfg);
  this->cfg.speed_factor = std::min(std::max(this->cfg.speed_factor, 0.0), 1.0);
  return true;
}

std::unique_ptr<common::RConfig> FR3::get_parameters() {
  std::unique_ptr<common::RConfig> ret = std::make_unique<common::RConfig>();
  *ret = this->cfg;
  return ret;
}

std::unique_ptr<common::RState> FR3::get_state() {
  return std::make_unique<common::RState>();
}

common::Pose FR3::get_cartesian_position() {
  return common::Pose(Eigen::Vector4d(this->mujoco_data->xquat),
                      Eigen::Vector3d(this->mujoco_data->xpos));
}

// TODO: should this return when it is done?
// should we check for collisions first?
// If yes, how do we return success or failure?
void FR3::set_joint_position(common::Vector7d const& q) {
  std::memcpy(this->mujoco_data->ctrl, q.data(), 7);
}

common::Vector7d FR3::get_joint_position() {
  return common::Vector7d(this->mujoco_data->qpos);
}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::set_cartesian_position(common::Pose const& pose) { 
  this->kinmdl->setPosition(common::Vector7d(this->mujoco_data->qpos));
  this->kinmdl->forwardPosition();
  this->ikmdl.addGoal(pose.affine_matrix(), 0);
  // TODO: IK failure error reporting
  bool success = this->ikmdl.solve();
  if (success) {
    this->kinmdl->forwardPosition();
    std::copy(this->kinmdl->getPosition().begin(), this->kinmdl->getPosition().end(), this->mujoco_data->ctrl);
  }
}

void FR3::render_loop() {
  constexpr int kMaxGeom = 20000;
  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjvPerturb pert;
  size_t current_sim = 0;

  mujoco::GlfwAdapter ui_adapter = mujoco::GlfwAdapter();

  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjv_makeScene(this->mjmdl.get(), &scn, kMaxGeom);
  mjv_defaultFreeCamera(this->mjmdl.get(), &cam);

  ui_adapter.RefreshMjrContext(this->mjmdl.get(), 1);
  mjuiState uistate = ui_adapter.state();
  std::memset(&uistate, 0, sizeof(mjuiState));
  uistate.nrect = 1;
  std::tie(uistate.rect[0].width, uistate.rect[0].height) =
      ui_adapter.GetFramebufferSize();
  if (!ui_adapter.IsGPUAccelerated()) {
    scn.flags[mjRND_SHADOW] = 0;
    scn.flags[mjRND_REFLECTION] = 0;
  }
  while (!(ui_adapter.ShouldCloseWindow() or this->exit_requested)) {
    std::tie(uistate.rect[0].width, uistate.rect[0].height) =
        ui_adapter.GetFramebufferSize();
    mjv_updateScene(this->mjmdl.get(), this->mujoco_data.get(), &opt, NULL,
                    &cam, mjCAT_ALL, &scn);
    mjr_render(uistate.rect[0], &scn, &ui_adapter.mjr_context());
    ui_adapter.SwapBuffers();
    ui_adapter.PollEvents();
  }
}
}  // namespace sim
}  // namespace rcs
