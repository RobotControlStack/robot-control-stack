#include "FR3.h"

#include <Eigen/src/Core/Map.h>
#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <tuple>

#include "common/Robot.h"
#include "glfw_adapter.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"

enum { SIM_KF_HOME = 0 };

double const POSE_REPEATABILITY = 0.0001;  // in meters

std::array arm_collision_geom_names = {
    "link0_c",  "link1_c",  "link2_c",  "link3_c", "link4_c",
    "link5_c0", "link5_c1", "link5_c2", "link6_c", "link7_c"};

char hand_collision_geom_name[] = "hand_c";

std::array finger_collision_geom_names = {
    "finger_0_left",
    "fingertip_pad_collision_1_left",
    "fingertip_pad_collision_2_left",
    "fingertip_pad_collision_3_left",
    "fingertip_pad_collision_4_left",
    "fingertip_pad_collision_5_left",
    "finger_0_right",
    "fingertip_pad_collision_1_right",
    "fingertip_pad_collision_2_right",
    "fingertip_pad_collision_3_right",
    "fingertip_pad_collision_4_right",
    "fingertip_pad_collision_5_right",
};

// TODO: Choose error handling strategy

namespace rcs {
namespace sim {
using std::endl;

FR3::FR3(const std::string& mjmdl, const std::string& rlmdl)
    : ikmdl(NULL), cfg(0, false, false) {
  char err[1024];
  this->mjmdl = std::shared_ptr<mjModel>(
      mj_loadXML(mjmdl.c_str(), NULL, err, sizeof(err)));
  if (!this->mjmdl.get()) {
    std::cerr << "Failed loading " << mjmdl << ":" << std::endl;
    std::cerr << err << std::endl;
    exit(EXIT_FAILURE); 
  }
  this->rlmdl = rl::mdl::UrdfFactory().create(rlmdl);
  this->mujoco_data = std::shared_ptr<mjData>(mj_makeData(this->mjmdl.get()));
  this->render_thread = std::jthread(std::bind(&FR3::render_loop, this));
  this->exit_requested = false;
  this->kinmdl = std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->rlmdl);
  this->ikmdl = rl::mdl::JacobianInverseKinematics(this->kinmdl.get());
  for (size_t i = 0; i < std::size(arm_collision_geom_names); ++i) {
    int id =
        mj_name2id(this->mjmdl.get(), mjOBJ_GEOM, arm_collision_geom_names[i]);
    if (id == -1) {
      std::cerr << "No geom named " << arm_collision_geom_names[i] << std::endl;
      exit(EXIT_FAILURE);
    }
    collision_geom_ids.insert(id);
  }
  this->hand_geom_id =
      mj_name2id(this->mjmdl.get(), mjOBJ_GEOM, hand_collision_geom_name);
  if (hand_geom_id == -1) {
    std::cerr << "No geom named \"hand_c\"" << std::endl;
    exit(EXIT_FAILURE);
  }
  for (size_t i = 0; i < std::size(finger_collision_geom_names); ++i) {
    int id = mj_name2id(this->mjmdl.get(), mjOBJ_GEOM,
                        finger_collision_geom_names[i]);
    if (id == -1) {
      std::cerr << "No geom named " << finger_collision_geom_names[i]
                << std::endl;
      exit(EXIT_FAILURE);
    }
    this->finger_collision_geom_ids.insert(id);
  }
  collision_geom_ids.insert(this->hand_geom_id);
  ikmdl.setDuration(std::chrono::milliseconds(300));
  mj_resetDataKeyframe(this->mjmdl.get(), this->mujoco_data.get(), SIM_KF_HOME);
  mj_step(this->mjmdl.get(), this->mujoco_data.get());
}

FR3::~FR3() { this->exit_requested = true; }

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
  int   hand_body_id  = mj_name2id(this->mjmdl.get(), mjOBJ_BODY, "hand");
  if (hand_body_id == -1) {
    std::cerr << "No body named \"hand\"" << std::endl;
    exit(EXIT_FAILURE);
  }
  return common::Pose(
      Eigen::Vector4d(this->mujoco_data->xquat + 4 * hand_geom_id),
      Eigen::Vector3d(this->mujoco_data->xpos + 3 * hand_geom_id));
}

void FR3::set_joint_position(common::Vector7d const& q) {
  std::copy(q.begin(), q.end(), this->mujoco_data->ctrl);
  this->wait_for_convergence(q);
}

common::Vector7d FR3::get_joint_position() {
  return common::Vector7d(this->mujoco_data->qpos);
}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::set_cartesian_position(common::Pose const& pose) {
  this->kinmdl->setPosition(common::Vector7d(this->mujoco_data->qpos));
  this->kinmdl->forwardPosition();
  this->ikmdl.addGoal(pose.affine_matrix(), 0);
  bool success = this->ikmdl.solve();
  if (not success) {
    std::cerr << "IK failed" << std::endl;
  }
  if (success) {
    this->kinmdl->forwardPosition();
    rl::math::Vector pos = this->kinmdl->getPosition();
    std::copy(pos.begin(), pos.end(), this->mujoco_data->ctrl);
    this->wait_for_convergence(pos);
  }
}

void FR3::wait_for_convergence(common::Vector7d target_angles) {
  bool arrived, moving = false;
  std::pair<common::Vector7d, Eigen::Map<common::Vector7d>> angles(
      this->mujoco_data->qpos, this->mujoco_data->qpos);
  while ((not arrived) or moving) {
    mj_step(this->mjmdl.get(), this->mujoco_data.get());
    moving =
        not angles.first.isApprox(angles.second, .5 * (std::numbers::pi / 180));
    arrived =
        angles.second.isApprox(target_angles, .5 * (std::numbers::pi / 180));
    angles.first = angles.second;
    if (this->collision()) {
      mj_resetDataKeyframe(this->mjmdl.get(), this->mujoco_data.get(),
                           SIM_KF_HOME);
      mj_step(this->mjmdl.get(), this->mujoco_data.get());
      return;
    }
    /* If we stopped moving but have not reached our target yet, check if there
     * is a collision with the gripper and stop if there is. */
    bool hand_collision = false;
    if (not moving and not arrived) {
      for (size_t i = 0; i < this->mujoco_data->ncon; ++i) {
        hand_collision =
            (this->finger_collision_geom_ids.contains(
                 this->mujoco_data->contact[i].geom[0]) ||
             this->finger_collision_geom_ids.contains(
                 this->mujoco_data->contact[i].geom[1]) ||
             this->mujoco_data->contact[i].geom[0] == this->hand_geom_id ||
             this->mujoco_data->contact[i].geom[1] ==
                 this->hand_geom_id);
        if (hand_collision) break;
      }
      if (hand_collision) {
        mj_resetDataKeyframe(this->mjmdl.get(), this->mujoco_data.get(),
                             SIM_KF_HOME);
        mj_step(this->mjmdl.get(), this->mujoco_data.get());
        break;
      }
    }
  }
}

bool FR3::collision() {
  for (size_t i = 0; i < this->mujoco_data->ncon; ++i) {
    if (this->collision_geom_ids.contains(
            this->mujoco_data->contact[i].geom[0]) ||
        this->collision_geom_ids.contains(
            this->mujoco_data->contact[i].geom[1])) {
      return true;
    }
  }
  return false;
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
  ui_adapter.SetVSync(true);
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
