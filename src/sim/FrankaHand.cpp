
#include "FrankaHand.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

struct {
  // TODO(juelg): add camera mount collision (first a name needs to be
  // added to the model)
  std::vector<std::string> collision_geoms{"hand_c", "d435i_collision",
                                           "finger_0_left", "finger_0_right"};

  std::vector<std::string> collision_geoms_fingers{"finger_0_left",
                                                   "finger_0_right"};
  std::string joint1 = "finger_joint1";
  std::string joint2 = "finger_joint2";
  std::string actuator = "actuator8";
} const gripper_names;

namespace rcs {
namespace sim {

FrankaHand::FrankaHand(std::shared_ptr<Sim> sim, const std::string &id,
                       const FHConfig &cfg)
    : sim{sim}, cfg{cfg}, id{id} {
  this->state = FHState();
  this->actuator_id = mj_name2id(this->sim->m, mjOBJ_ACTUATOR,
                                 (gripper_names.actuator + "_" + id).c_str());
  if (this->actuator_id == -1) {
    throw std::runtime_error(
        std::string("No actuator named " + gripper_names.actuator));
  }
  // this->tendon_id =
  //     mj_name2id(this->sim->m, mjOBJ_TENDON, ("split_" + id).c_str());
  this->joint_id_1 = this->sim->m->jnt_qposadr[mj_name2id(
      this->sim->m, mjOBJ_JOINT, (gripper_names.joint1 + "_" + id).c_str())];
  if (this->joint_id_1 == -1) {
    throw std::runtime_error(
        std::string("No joint named " + gripper_names.joint1));
  }
  this->joint_id_2 = this->sim->m->jnt_qposadr[mj_name2id(
      this->sim->m, mjOBJ_JOINT, (gripper_names.joint2 + "_" + id).c_str())];
  if (this->joint_id_2 == -1) {
    throw std::runtime_error(
        std::string("No joint named " + gripper_names.joint2));
  }
  // Collision geoms
  this->add_collision_geoms(gripper_names.collision_geoms, this->cgeom, false);
  this->add_collision_geoms(gripper_names.collision_geoms_fingers, this->cfgeom,
                            false);

  this->sim->register_all_cb(std::bind(&FrankaHand::convergence_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->sim->register_cb(std::bind(&FrankaHand::collision_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->m_reset();
}

FrankaHand::~FrankaHand() {}

void FrankaHand::add_collision_geoms(const std::vector<std::string> &cgeoms_str,
                                     std::set<size_t> &cgeoms_set,
                                     bool clear_before) {
  if (clear_before) {
    cgeoms_set.clear();
  }
  for (size_t i = 0; i < std::size(cgeoms_str); ++i) {
    std::string name = cgeoms_str[i] + "_" + id;

    int coll_id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
    if (coll_id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    cgeoms_set.insert(coll_id);
  }
}

bool FrankaHand::set_parameters(const FHConfig &cfg) {
  this->cfg = cfg;
  this->add_collision_geoms(cfg.ignored_collision_geoms,
                            this->ignored_collision_geoms, true);
  return true;
}

FHConfig *FrankaHand::get_parameters() {
  // copy config to heap
  FHConfig *cfg = new FHConfig();
  *cfg = this->cfg;
  return cfg;
}

FHState *FrankaHand::get_state() {
  FHState *state = new FHState();
  *state = this->state;
  return state;
}
void FrankaHand::set_normalized_width(double width, double force) {
  if (width < 0 || width > 1 || force < 0) {
    throw std::invalid_argument(
        "width must be between 0 and 1, force must be positive");
  }
  this->state.last_commanded_width = width;
  this->sim->d->ctrl[this->actuator_id] = (mjtNum)width * this->MAX_WIDTH;

  // we ignore force for now
  // this->sim->d->actuator_force[this->gripper_id] = 0;
}
double FrankaHand::get_normalized_width() {
  // TODO: maybe we should use the mujoco sensors? Not sure what the difference
  // is between reading out from qpos and reading from the sensors.
  double width = this->sim->d->qpos[this->joint_id_1] / this->MAX_JOINT_WIDTH;
  // sometimes the joint is slightly outside of the bounds
  if (width < 0) {
    width = 0;
  } else if (width > 1) {
    width = 1;
  }
  return width;
}

bool FrankaHand::collision_callback() {
  this->state.collision = false;
  for (size_t i = 0; i < this->sim->d->ncon; ++i) {
    if (this->cfgeom.contains(this->sim->d->contact[i].geom[0]) &&
        this->cfgeom.contains(this->sim->d->contact[i].geom[1])) {
      // ignore collisions between fingers
      continue;
    }

    if ((this->cgeom.contains(this->sim->d->contact[i].geom[0]) or
         this->cgeom.contains(this->sim->d->contact[i].geom[1])) and
        // ignore all collision with ignored objects with frankahand
        // not just fingers
        not(this->ignored_collision_geoms.contains(
                this->sim->d->contact[i].geom[1]) or
            this->ignored_collision_geoms.contains(
                this->sim->d->contact[i].geom[1]))) {
      this->state.collision = true;
      break;
    }
  }
  return this->state.collision;
}

bool FrankaHand::is_grasped() {
  double width = this->get_normalized_width();

  if (this->state.last_commanded_width - this->cfg.epsilon_inner < width &&
      width < this->state.last_commanded_width + this->cfg.epsilon_outer) {
    // this is the libfranka logic to decide whether an object is grasped
    return true;
  }
  return false;
}

bool FrankaHand::convergence_callback() {
  double w = get_normalized_width();
  this->state.is_moving = std::abs(this->state.last_width - w) >
                          0.001 / this->MAX_WIDTH;  // 1mm tolerance
  this->state.last_width = w;
  return not this->state.is_moving;
}

void FrankaHand::grasp() { this->shut(); }

void FrankaHand::open() { this->set_normalized_width(1); }
void FrankaHand::shut() { this->set_normalized_width(0); }

void FrankaHand::m_reset() {
  this->state = FHState();
  this->state.max_unnormalized_width = this->MAX_WIDTH;
  // reset state hard
  this->sim->d->qpos[this->joint_id_1] = this->MAX_JOINT_WIDTH;
  this->sim->d->qpos[this->joint_id_2] = this->MAX_JOINT_WIDTH;
  this->sim->d->ctrl[this->actuator_id] = this->MAX_WIDTH;
}

void FrankaHand::reset() { this->m_reset(); }
}  // namespace sim
}  // namespace rcs
