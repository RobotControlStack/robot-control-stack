
#include "SimGripper.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

namespace rcs {
namespace sim {

SimGripper::SimGripper(std::shared_ptr<Sim> sim, const SimGripperConfig &cfg)
    : sim{sim}, cfg{cfg} {
  this->state = SimGripperState();
  this->actuator_id =
      mj_name2id(this->sim->m, mjOBJ_ACTUATOR, this->cfg.actuator.c_str());
  if (this->actuator_id == -1) {
    throw std::runtime_error(
        std::string("No actuator named " + this->cfg.actuator));
  }
  // this->tendon_id =
  //     mj_name2id(this->sim->m, mjOBJ_TENDON, ("split_" + id).c_str());
  this->joint_id = this->sim->m->jnt_qposadr[mj_name2id(
      this->sim->m, mjOBJ_JOINT, this->cfg.joint.c_str())];
  if (this->joint_id == -1) {
    throw std::runtime_error(std::string("No joint named " + this->cfg.joint));
  }
  // Collision geoms
  this->add_collision_geoms(this->cfg.collision_geoms, this->cgeom, false);
  this->add_collision_geoms(this->cfg.collision_geoms_fingers, this->cfgeom,
                            false);

  this->sim->register_all_cb(std::bind(&SimGripper::convergence_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->sim->register_any_cb(std::bind(&SimGripper::collision_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->m_reset();
}

SimGripper::~SimGripper() {}

void SimGripper::add_collision_geoms(const std::vector<std::string> &cgeoms_str,
                                     std::set<size_t> &cgeoms_set,
                                     bool clear_before) {
  if (clear_before) {
    cgeoms_set.clear();
  }
  for (size_t i = 0; i < std::size(cgeoms_str); ++i) {
    std::string name = cgeoms_str[i];

    int coll_id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
    if (coll_id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    cgeoms_set.insert(coll_id);
  }
}

bool SimGripper::set_config(const SimGripperConfig &cfg) {
  this->cfg = cfg;
  this->add_collision_geoms(cfg.ignored_collision_geoms,
                            this->ignored_collision_geoms, true);
  return true;
}

SimGripperConfig *SimGripper::get_config() {
  // copy config to heap
  SimGripperConfig *cfg = new SimGripperConfig();
  *cfg = this->cfg;
  return cfg;
}

SimGripperState *SimGripper::get_state() {
  SimGripperState *state = new SimGripperState();
  *state = this->state;
  return state;
}
void SimGripper::set_normalized_width(double width, double force) {
  if (width < 0 || width > 1 || force < 0) {
    throw std::invalid_argument(
        "width must be between 0 and 1, force must be positive");
  }
  this->state.last_commanded_width = width;
  this->sim->d->ctrl[this->actuator_id] =
      (mjtNum)width *
          (this->cfg.max_actuator_width - this->cfg.min_actuator_width) +
      this->cfg.min_actuator_width;

  // we ignore force for now
  // this->sim->d->actuator_force[this->gripper_id] = 0;
}
double SimGripper::get_normalized_width() {
  // TODO: maybe we should use the mujoco sensors? Not sure what the difference
  // is between reading out from qpos and reading from the sensors.
  double width =
      (this->sim->d->qpos[this->joint_id] - this->cfg.min_joint_width) /
      (this->cfg.max_joint_width - this->cfg.min_joint_width);
  // sometimes the joint is slightly outside of the bounds
  if (width < 0) {
    width = 0;
  } else if (width > 1) {
    width = 1;
  }
  return width;
}

bool SimGripper::collision_callback() {
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

bool SimGripper::is_grasped() {
  double width = this->get_normalized_width();

  if (this->state.last_commanded_width - this->cfg.epsilon_inner < width &&
      width < this->state.last_commanded_width + this->cfg.epsilon_outer) {
    // this is the libfranka logic to decide whether an object is grasped
    return true;
  }
  return false;
}

bool SimGripper::convergence_callback() {
  double w = get_normalized_width();
  this->state.is_moving =
      std::abs(this->state.last_width - w) >
      0.001 * (this->cfg.max_actuator_width -
               this->cfg.min_actuator_width);  // 0.1% tolerance
  this->state.last_width = w;
  return not this->state.is_moving;
}

void SimGripper::grasp() { this->shut(); }

void SimGripper::open() { this->set_normalized_width(1); }
void SimGripper::shut() { this->set_normalized_width(0); }

void SimGripper::m_reset() {
  this->state = SimGripperState();
  // reset state hard
  this->sim->d->qpos[this->joint_id] = this->cfg.max_joint_width;
  this->sim->d->ctrl[this->actuator_id] = this->cfg.max_actuator_width;
}

void SimGripper::reset() { this->m_reset(); }
}  // namespace sim
}  // namespace rcs
