
#include "FrankaHand.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

struct {
  std::array<std::string, 2> collision_geoms{"hand_c", "d435i_collision"};
  std::string joint = "finger_joint1";
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
  this->joint_id = this->sim->m->jnt_qposadr[mj_name2id(
      this->sim->m, mjOBJ_JOINT, (gripper_names.joint + "_" + id).c_str())];
  if (this->joint_id == -1) {
    throw std::runtime_error(
        std::string("No joint named " + gripper_names.joint));
  }
  // Collision geoms
  for (size_t i = 0; i < std::size(gripper_names.collision_geoms); ++i) {
    std::string name = gripper_names.collision_geoms[i] + "_" + id;

    int coll_id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
    if (coll_id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    this->cgeom.insert(coll_id);
  }

  this->sim->register_cb(std::bind(&FrankaHand::is_moving_callback, this),
                         this->cfg.seconds_between_callbacks);
  this->sim->register_all_cb(std::bind(&FrankaHand::convergence_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->sim->register_any_cb(std::bind(&FrankaHand::collision_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->m_reset();
}

FrankaHand::~FrankaHand() {}

bool FrankaHand::set_parameters(const FHConfig &cfg) { return true; }

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
  // return this->sim->d->qpos[this->sim->m->tendon_adr[this->tendon_id]] /
  //        this->MAX_WIDTH;
  double width = this->sim->d->qpos[this->joint_id] / this->MAX_JOINT_WIDTH;
  // sometimes the joint is slighly outside of the bounds
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
    if (this->cgeom.contains(this->sim->d->contact[i].geom[0]) ||
        this->cgeom.contains(this->sim->d->contact[i].geom[1])) {
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

void FrankaHand::is_moving_callback() {
  double w = get_normalized_width();
  this->state.is_moving = std::abs(this->state.last_width - w) <
                          0.001 / this->MAX_WIDTH;  // 1mm tolerance
  this->state.last_width = w;
}

bool FrankaHand::convergence_callback() { return this->state.is_moving; }

void FrankaHand::grasp() { this->shut(); }

void FrankaHand::open() { this->set_normalized_width(1); }
void FrankaHand::shut() { this->set_normalized_width(0); }

void FrankaHand::m_reset() {
  this->state = FHState();
  this->state.max_unnormalized_width = this->MAX_WIDTH;
  // reset state hard
  this->sim->d->qpos[this->joint_id] = this->MAX_JOINT_WIDTH;
}

void FrankaHand::reset() { this->m_reset(); }
}  // namespace sim
}  // namespace rcs