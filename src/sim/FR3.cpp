#include "FR3.h"

#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "common/Robot.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"

struct {
  std::array<std::string, 10> arm_collision_geoms{
      "link0_c",  "link1_c",  "link2_c",  "link3_c", "link4_c",
      "link5_c0", "link5_c1", "link5_c2", "link6_c", "link7_c"};
  std::array<std::string, 7> joints = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
  std::array<std::string, 7> actuators = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
} const model_names;

namespace rcs {
namespace sim {

FR3::FR3(Sim sim, std::string& id, std::shared_ptr<rl::mdl::Model> rlmdl)
    : sim{sim}, id{id}, rl{.mdl = rlmdl}, cfg{}, state{} {
  this->rl.kin = std::dynamic_pointer_cast<rl::mdl::Kinematic>(rlmdl);
  this->rl.ik =
      std::make_shared<rl::mdl::JacobianInverseKinematics>(this->rl.kin.get());
  this->rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration_in_milliseconds));
  this->init_ids();
  this->sim.register_cb(std::bind(&FR3::collision_callback, this),
                        this->cfg.seconds_between_callbacks);
  this->sim.register_cb(std::bind(&FR3::is_arrived_callback, this),
                        this->cfg.seconds_between_callbacks);
  this->sim.register_cb(std::bind(&FR3::is_moving_callback, this),
                        this->cfg.seconds_between_callbacks);
  this->sim.register_convergence_cb(std::bind(&FR3::convergence_callback, this),
                                    this->cfg.seconds_between_callbacks);
  this->reset();
}

void FR3::init_ids() {
  std::string name;
  // Collision geoms
  for (size_t i = 0; i < std::size(model_names.arm_collision_geoms); ++i) {
    name = model_names.arm_collision_geoms[i];
    name.append("_");
    name.append(this->id);
    auto id = mj_name2id(this->sim.m, mjOBJ_GEOM, name.c_str());
    if (id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    this->ids.cgeom.insert(id);
  }
  // Sites
  name = "attachment_site_";
  name.append(this->id);
  this->ids.attachment_site = mj_name2id(this->sim.m, mjOBJ_SITE, name.c_str());
  if (this->ids.attachment_site == -1) {
    throw std::runtime_error(std::string("No site named " + name));
  }
  // Joints
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    name = model_names.joints[i];
    name.append("_");
    name.append(this->id);
    this->ids.ctrl[i] = mj_name2id(this->sim.m, mjOBJ_JOINT, name.c_str());
    if (this->ids.ctrl[i] == -1) {
      throw std::runtime_error(std::string("No joint named " + name));
    }
  }
  // Actuators
  for (size_t i = 0; i < std::size(model_names.actuators); ++i) {
    name = model_names.actuators[i];
    name.append("_");
    name.append(this->id);
    this->ids.ctrl[i] = mj_name2id(this->sim.m, mjOBJ_ACTUATOR, name.c_str());
    if (this->ids.ctrl[i] == -1) {
      throw std::runtime_error(std::string("No actuator named " + name));
    }
  }
}

bool FR3::set_parameters(const FR3Config& cfg) {
  this->cfg = cfg;
  this->rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration_in_milliseconds));
  return true;
}

FR3Config* FR3::get_parameters() {
  FR3Config* cfg = new FR3Config();
  *cfg = this->cfg;
  return cfg;
}

FR3State* FR3::get_state() {
  FR3State* state = new FR3State();
  *state = this->state;
  return state;
}

common::Pose FR3::get_cartesian_position() {
  auto rotation =
      Eigen::Matrix3d(this->sim.d->site_xmat + 9 * this->ids.attachment_site);
  auto translation =
      Eigen::Vector3d(this->sim.d->site_xpos + 3 * this->ids.attachment_site);
  auto attachment_site = common::Pose(rotation, translation);
  return attachment_site * this->cfg.tcp_offset;
}

void FR3::set_joint_position(const common::Vector7d& q) {
  this->state.target_angles = q;
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    this->sim.d->ctrl[this->sim.m->jnt_qposadr[this->ids.joints[i]]] = q[i];
  }
}

common::Vector7d FR3::get_joint_position() {
  common::Vector7d q;
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    q[i] = this->sim.d->ctrl[this->sim.m->jnt_qposadr[this->ids.joints[i]]];
  }
  return q;
}

void FR3::set_cartesian_position(const common::Pose& pose) {
  this->rl.kin->setPosition(this->get_joint_position());
  this->rl.kin->forwardPosition();
  auto new_pose = pose * this->cfg.tcp_offset.inverse();
  this->rl.ik->addGoal(new_pose.affine_matrix(), 0);
  if (this->rl.ik->solve()) {
    this->state.ik_success = false;
    this->rl.kin->forwardPosition();
    this->set_joint_position(this->rl.kin->getPosition());
  } else {
    this->state.ik_success = false;
  }
}
void FR3::is_moving_callback() {
  auto current_angles = this->get_joint_position();
  this->state.is_moving = not this->state.previous_angles.isApprox(
      current_angles, this->cfg.tolerance);
}

void FR3::is_arrived_callback() {
  auto current_angles = this->get_joint_position();
  this->state.is_arrived =
      this->state.target_angles.isApprox(current_angles, this->cfg.tolerance);
}

void FR3::collision_callback() {
  this->state.collision = false;
  for (size_t i = 0; i < this->sim.d->ncon; ++i) {
    if (this->ids.cgeom.contains(this->sim.d->contact[i].geom[0]) ||
        this->ids.cgeom.contains(this->sim.d->contact[i].geom[0])) {
      this->state.collision = true;
    }
  }
}

bool FR3::convergence_callback() {
  /* Not moving anymore but not arrived → arm is stuck */
  if (not this->state.is_moving and not this->state.is_arrived) {
    return true;
  }
  /* Other we are done when we arrived and stopped moving */
  return this->state.is_arrived and not this->state.is_moving;
}
}  // namespace sim
}  // namespace rcs
