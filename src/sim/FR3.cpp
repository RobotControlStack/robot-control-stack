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

const rcs::common::Vector7d q_home((rcs::common::Vector7d() << 0, -M_PI_4, 0,
                                    -3 * M_PI_4, 0, M_PI_2, M_PI_4)
                                       .finished());

struct {
  std::array<std::string, 8> arm_collision_geoms{
      "fr3_link0_collision", "fr3_link1_collision", "fr3_link2_collision",
      "fr3_link3_collision", "fr3_link4_collision", "fr3_link5_collision",
      "fr3_link6_collision", "fr3_link7_collision"};
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

FR3::FR3(std::shared_ptr<Sim> sim, std::string& id,
         std::shared_ptr<rl::mdl::Model> rlmdl)
    : sim{sim}, id{id}, rl{.mdl = rlmdl}, cfg{}, state{} {
  this->rl.kin = std::dynamic_pointer_cast<rl::mdl::Kinematic>(rlmdl);
  this->rl.ik =
      std::make_shared<rl::mdl::JacobianInverseKinematics>(this->rl.kin.get());
  this->rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration_in_milliseconds));
  this->init_ids();
  this->sim->register_cb(std::bind(&FR3::collision_callback, this),
                         this->cfg.seconds_between_callbacks);
  this->sim->register_cb(std::bind(&FR3::is_arrived_callback, this),
                         this->cfg.seconds_between_callbacks);
  this->sim->register_cb(std::bind(&FR3::is_moving_callback, this),
                         this->cfg.seconds_between_callbacks);
  this->sim->register_all_cb(std::bind(&FR3::convergence_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->sim->register_any_cb(std::bind(&FR3::collision_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->reset();
}

FR3::~FR3() {}

void FR3::move_home() { this->set_joint_position(q_home); }

void FR3::init_ids() {
  std::string name;
  // Collision geoms
  for (size_t i = 0; i < std::size(model_names.arm_collision_geoms); ++i) {
    name = model_names.arm_collision_geoms[i];
    name.append("_");
    name.append(this->id);
    int id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
    if (id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    this->ids.cgeom.insert(id);
  }
  // Sites
  name = "attachment_site_";
  name.append(this->id);
  this->ids.attachment_site =
      mj_name2id(this->sim->m, mjOBJ_SITE, name.c_str());
  if (this->ids.attachment_site == -1) {
    throw std::runtime_error(std::string("No site named " + name));
  }
  // Joints
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    name = model_names.joints[i];
    name.append("_");
    name.append(this->id);
    this->ids.joints[i] = mj_name2id(this->sim->m, mjOBJ_JOINT, name.c_str());
    if (this->ids.joints[i] == -1) {
      throw std::runtime_error(std::string("No joint named " + name));
    }
  }
  // Actuators
  for (size_t i = 0; i < std::size(model_names.actuators); ++i) {
    name = model_names.actuators[i];
    name.append("_");
    name.append(this->id);
    this->ids.actuators[i] =
        mj_name2id(this->sim->m, mjOBJ_ACTUATOR, name.c_str());
    if (this->ids.actuators[i] == -1) {
      throw std::runtime_error(std::string("No actuator named " + name));
    }
  }
}

bool FR3::set_parameters(const FR3Config& cfg) {
  this->cfg = cfg;
  this->rl.ik->setDuration(
      std::chrono::milliseconds(this->cfg.ik_duration_in_milliseconds));
  this->state.inverse_tcp_offset = cfg.tcp_offset.inverse();
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
  Eigen::Matrix3d rotation(this->sim->d->site_xmat +
                           9 * this->ids.attachment_site);
  Eigen::Vector3d translation(this->sim->d->site_xpos +
                              3 * this->ids.attachment_site);
  common::Pose attachment_site(rotation, translation);
  // TODO: Why do we have to take the inverse here? Should be the normal offset
  return attachment_site * this->state.inverse_tcp_offset;
}

void FR3::set_joint_position(const common::Vector7d& q) {
  this->state.target_angles = q;
  this->state.previous_angles = this->get_joint_position();
  this->state.is_moving = true;
  this->state.is_arrived = false;
  for (size_t i = 0; i < std::size(this->ids.actuators); ++i) {
    this->sim->d->ctrl[this->ids.actuators[i]] = q[i];
  }
}

common::Vector7d FR3::get_joint_position() {
  common::Vector7d q;
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    q[i] = this->sim->d->qpos[this->sim->m->jnt_qposadr[this->ids.joints[i]]];
  }
  return q;
}

void FR3::set_cartesian_position(const common::Pose& pose) {
  this->rl.kin->setPosition(this->get_joint_position());
  this->rl.kin->forwardPosition();
  // TODO: Why do we have to take the tcp offset and not the inverse here?
  // Should be the opposite.
  rcs::common::Pose new_pose = pose * this->cfg.tcp_offset;
  this->rl.ik->addGoal(new_pose.affine_matrix(), 0);
  if (this->rl.ik->solve()) {
    this->state.ik_success = true;
    this->rl.kin->forwardPosition();
    this->set_joint_position(this->rl.kin->getPosition());
  } else {
    this->state.ik_success = false;
  }
}
void FR3::is_moving_callback() {
  common::Vector7d current_angles = this->get_joint_position();
  this->state.is_moving =
      not this->state.previous_angles.isApprox(current_angles, 0.0001);
  this->state.previous_angles = current_angles;
}

void FR3::is_arrived_callback() {
  common::Vector7d current_angles = this->get_joint_position();
  this->state.is_arrived = this->state.target_angles.isApprox(
      current_angles, this->cfg.joint_rotational_tolerance);
}

bool FR3::collision_callback() {
  this->state.collision = false;
  for (size_t i = 0; i < this->sim->d->ncon; ++i) {
    if (this->ids.cgeom.contains(this->sim->d->contact[i].geom[0]) ||
        this->ids.cgeom.contains(this->sim->d->contact[i].geom[1])) {
      this->state.collision = true;
      break;
    }
  }
  return this->state.collision;
}

bool FR3::convergence_callback() {
  /* When ik failed, the robot is not doing anything */
  if (not this->state.ik_success) {
    return true;
  }
  /* Otherwise we are done when we arrived and stopped moving */
  return this->state.is_arrived and not this->state.is_moving;
}

void FR3::reset() {
  for (size_t i = 0; i < std::size(this->ids.joints); ++i) {
    size_t jnt_id = this->ids.joints[i];
    size_t jnt_qposadr = this->sim->m->jnt_qposadr[jnt_id];
    this->sim->d->qpos[jnt_qposadr] = q_home[i];
  }
}
}  // namespace sim
}  // namespace rcs
