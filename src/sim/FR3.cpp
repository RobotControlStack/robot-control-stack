#include "FR3.h"

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "common/Robot.h"
#include "glfw_adapter.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"

enum { SIM_KF_HOME = 0 };

double const POSE_REPEATABILITY = 0.0001;  // in meters

struct {
  std::vector<std::string> arm = {"link0_c", "link1_c",  "link2_c",  "link3_c",
                                  "link4_c", "link5_c0", "link5_c1", "link5_c2",
                                  "link6_c", "link7_c"};
  std::vector<std::string> hand = {"hand_c"};
  std::vector<std::string> gripper = {
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
} const cgeom_names;

static void init_geom_ids(std::set<size_t>& ids,
                          std::vector<std::string> const& names,
                          mjModel const& m) {
  for (auto const& name : names) {
    int id = mj_name2id(&m, mjOBJ_GEOM, name.c_str());
    if (id == -1)
      throw std::runtime_error(std::string("No geom named ") + name);
    ids.insert(id);
  }
}

template <typename T>
std::set<T> set_union(const std::set<T>& a, const std::set<T>& b) {
  std::set<T> result = a;
  result.insert(b.begin(), b.end());
  return result;
}

namespace rcs {
namespace sim {
using std::endl;

FR3::FR3(mjModel* mjmdl, mjData* mjdata, std::shared_ptr<rl::mdl::Model> rlmdl,
         std::optional<bool> render)
    : models{.mj = {.mdl = mjmdl, .data = mjdata},
             .rl =
                 {
                     .mdl = rlmdl,
                     .kin =
                         std::dynamic_pointer_cast<rl::mdl::Kinematic>(rlmdl),
                 }},
      exit_requested(false) {
  this->models.rl.ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(
      this->models.rl.kin.get());
  /* Initialize collision geom id map */
  init_geom_ids(this->cgeom_ids.arm, cgeom_names.arm, *this->models.mj.mdl);
  init_geom_ids(this->cgeom_ids.gripper, cgeom_names.gripper,
                *this->models.mj.mdl);
  init_geom_ids(this->cgeom_ids.hand, cgeom_names.hand, *this->models.mj.mdl);
  this->models.rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration));
  /* Initialize sim */
  this->reset();
  if (render.value_or(true))
    this->render_thread = std::jthread(std::bind(&FR3::render_loop, this));
}

FR3::FR3(const std::string& mjmdl, const std::string& rlmdl,
         std::optional<bool> render)
    : models(), exit_requested(false) {
  /* Load models */
  char err[1024];
  this->models.mj.mdl = mj_loadXML(mjmdl.c_str(), NULL, err, sizeof(err));
  if (!this->models.mj.mdl) throw std::runtime_error(err);
  /* Throws in case of a failure */
  this->models.rl.mdl = rl::mdl::UrdfFactory().create(rlmdl);
  /* The next line can call exit(EXIT_FAILURE). What happens in such a case and
   * what should we do about it? */
  this->models.mj.data = mj_makeData(this->models.mj.mdl);
  this->models.rl.kin =
      std::dynamic_pointer_cast<rl::mdl::Kinematic>(this->models.rl.mdl);
  this->models.rl.ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(
      this->models.rl.kin.get());
  /* Initialize collision geom id map */
  init_geom_ids(this->cgeom_ids.arm, cgeom_names.arm, *this->models.mj.mdl);
  init_geom_ids(this->cgeom_ids.gripper, cgeom_names.gripper,
                *this->models.mj.mdl);
  init_geom_ids(this->cgeom_ids.hand, cgeom_names.hand, *this->models.mj.mdl);
  this->models.rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration));
  /* Initialize sim */
  this->reset();
  if (render.value_or(true))
    this->render_thread = std::jthread(std::bind(&FR3::render_loop, this));
}

FR3::~FR3() { this->exit_requested = true; }

bool FR3::set_parameters(common::RConfig const& cfg) {
  this->cfg = dynamic_cast<FR3Config const&>(cfg);
  this->models.rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration));
  return true;
}

std::unique_ptr<common::RConfig> FR3::get_parameters() {
  std::unique_ptr<common::RConfig> ret = std::make_unique<common::RConfig>();
  *ret = this->cfg;
  return ret;
}

std::unique_ptr<common::RState> FR3::get_state() {
  std::unique_ptr<FR3State> ret = std::make_unique<FR3State>();
  *ret = this->state;
  return ret;
}

common::Pose FR3::get_cartesian_position() {
  int tcp_id = mj_name2id(this->models.mj.mdl, mjOBJ_SITE, "tcp");
  if (tcp_id == -1) throw std::runtime_error("No site named \"tcp\"");
  return common::Pose(
      Eigen::Matrix3d(this->models.mj.data->site_xmat + 9 * tcp_id),
      Eigen::Vector3d(this->models.mj.data->site_xpos + 3 * tcp_id));
}

void FR3::set_joint_position(common::Vector7d const& q) {
  std::copy(q.begin(), q.end(), this->models.mj.data->ctrl);
  this->wait_for_convergence(q);
}

common::Vector7d FR3::get_joint_position() {
  return common::Vector7d(this->models.mj.data->qpos);
}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::set_cartesian_position(common::Pose const& pose) {
  this->models.rl.kin->setPosition(
      common::Vector7d(this->models.mj.data->qpos));
  this->models.rl.kin->forwardPosition();
  this->models.rl.ik->addGoal(pose.affine_matrix(), 0);
  if (this->models.rl.ik->solve()) {
    this->models.rl.kin->forwardPosition();
    rl::math::Vector pos = this->models.rl.kin->getPosition();
    std::copy(pos.begin(), pos.end(), this->models.mj.data->ctrl);
    this->wait_for_convergence(pos);
  } else {
    this->state.ik_success = false;
  }
}

void FR3::wait_for_convergence(common::Vector7d target_angles) {
  bool arrived = false;
  bool moving = false;
  std::pair<common::Vector7d, Eigen::Map<common::Vector7d>> angles(
      this->models.mj.data->qpos, this->models.mj.data->qpos);
  // TODO: move to config
  double tolerance = .5 * (std::numbers::pi / 180);

  std::pair<common::Pose, common::Pose> marker(this->get_cartesian_position(),
                                               this->get_cartesian_position());
  float marker_col[4] = {0, 1, 0, 1};
  int marker_count = 0;

  while ((not arrived) or moving) {
    mj_step(this->models.mj.mdl, this->models.mj.data);

    if (this->cfg.trajectory_trace) {
      if (marker_count == 0) {
        marker.first = marker.second;
        marker.second = this->get_cartesian_position();
        this->add_line(marker.first, marker.second, 0.003, marker_col);
      }
      marker_count = ++marker_count % 10;
    }

    // TODO: Replace by actual clock synchronization
    if (this->cfg.realtime)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

    moving = not angles.first.isApprox(angles.second, tolerance);
    arrived = angles.second.isApprox(target_angles, tolerance);
    angles.first = angles.second;
    if (this->collision(this->cgeom_ids.arm)) {
      this->state.collision = true;
      return this->reset();
    }
    if (not moving and not arrived and
        this->collision(
            set_union(this->cgeom_ids.hand, this->cgeom_ids.gripper))) {
      return this->reset();
    }
  }
}

bool FR3::collision(std::set<size_t> const& geom_ids) {
  for (size_t i = 0; i < this->models.mj.data->ncon; ++i) {
    if (geom_ids.contains(this->models.mj.data->contact[i].geom[0]) ||
        geom_ids.contains(this->models.mj.data->contact[i].geom[1]))
      return true;
  }
  return false;
}

std::list<mjvGeom>::iterator FR3::add_sphere(const common::Pose& pose,
                                             double size, const float* rgba) {
  mjvGeom newgeom;
  mjtNum* pos = pose.translation().data();
  mjv_initGeom(&newgeom, mjGEOM_SPHERE, &size, pos, NULL, rgba);
  return this->markers.insert(this->markers.end(), newgeom);
}

std::list<mjvGeom>::iterator FR3::add_line(const common::Pose& pose_from,
                                           const common::Pose& pose_to,
                                           double size, const float* rgba) {
  mjvGeom newgeom;
  mjtNum* from = pose_from.translation().data();
  mjtNum* to = pose_to.translation().data();
  mjv_initGeom(&newgeom, mjGEOM_CAPSULE, &size, NULL, NULL, rgba);
  mjv_connector(&newgeom, mjGEOM_CAPSULE, size, from, to);
  return this->markers.insert(this->markers.end(), newgeom);
}

void FR3::remove_marker(std::list<mjvGeom>::iterator& it) {
  this->markers.erase(it);
}

void FR3::clear_markers() { this->markers.clear(); }

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
  mjv_makeScene(this->models.mj.mdl, &scn, kMaxGeom);
  mjv_defaultFreeCamera(this->models.mj.mdl, &cam);

  ui_adapter.RefreshMjrContext(this->models.mj.mdl, 1);
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
    mjv_updateScene(this->models.mj.mdl, this->models.mj.data, &opt, NULL, &cam,
                    mjCAT_ALL, &scn);

    // Add markers
    for (mjvGeom marker : this->markers) {
      if (scn.ngeom >= scn.maxgeom) {
        mj_warning(this->models.mj.data, mjWARN_VGEOMFULL, scn.maxgeom);
        break;
      }
      mjvGeom* thisgeom = scn.geoms + scn.ngeom;
      *thisgeom = marker;
      thisgeom->segid = scn.ngeom;
      scn.ngeom++;
    }

    mjr_render(uistate.rect[0], &scn, &ui_adapter.mjr_context());
    ui_adapter.SwapBuffers();
    ui_adapter.PollEvents();
  }
}
void FR3::reset() {
  mj_resetDataKeyframe(this->models.mj.mdl, this->models.mj.data, SIM_KF_HOME);
  mj_step(this->models.mj.mdl, this->models.mj.data);
}
}  // namespace sim
}  // namespace rcs
