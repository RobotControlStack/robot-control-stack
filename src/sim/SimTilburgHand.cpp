
#include "SimTilburgHand.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <string>
#include <tuple>

namespace rcs {
namespace sim {

SimTilburgHand::SimTilburgHand(std::shared_ptr<Sim> sim,
                               const SimTilburgHandConfig &cfg)
    : sim{sim}, cfg{cfg} {
  this->state = SimTilburgHandState();

  // Initialize actuator and joint IDs
  for (int i = 0; i < this->n_joints; ++i) {
    // actuators
    this->actuator_ids[i] = mj_name2id(this->sim->m, mjOBJ_ACTUATOR,
                                       this->cfg.actuators[i].c_str());
    if (this->actuator_ids[i] == -1) {
      throw std::runtime_error(
          std::string("No actuator named " + this->cfg.actuators[i]));
    }
    // joints
    this->joint_ids[i] = this->sim->m->jnt_qposadr[mj_name2id(
        this->sim->m, mjOBJ_JOINT, this->cfg.joints[i].c_str())];
    if (this->joint_ids[i] == -1) {
      throw std::runtime_error(
          std::string("No joint named " + this->cfg.joints[i]));
    }
  }

  // Collision geoms
  // this->add_collision_geoms(this->cfg.collision_geoms, this->cgeom, false);
  // this->add_collision_geoms(this->cfg.collision_geoms_fingers, this->cfgeom,
  //                           false);

  this->sim->register_all_cb(
      std::bind(&SimTilburgHand::convergence_callback, this),
      this->cfg.seconds_between_callbacks);
  this->sim->register_any_cb(
      std::bind(&SimTilburgHand::collision_callback, this),
      this->cfg.seconds_between_callbacks);
  this->m_reset();
}

SimTilburgHand::~SimTilburgHand() {}

// void SimTilburgHand::add_collision_geoms(
//     const std::vector<std::string> &cgeoms_str, std::set<size_t> &cgeoms_set,
//     bool clear_before) {
//   if (clear_before) {
//     cgeoms_set.clear();
//   }
//   for (size_t i = 0; i < std::size(cgeoms_str); ++i) {
//     std::string name = cgeoms_str[i];

//     int coll_id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
//     if (coll_id == -1) {
//       throw std::runtime_error(std::string("No geom named " + name));
//     }
//     cgeoms_set.insert(coll_id);
//   }
// }

bool SimTilburgHand::set_parameters(const SimTilburgHandConfig &cfg) {
  auto current_grasp_type = this->cfg.grasp_type;
  this->cfg = cfg;
  if (!this->set_grasp_type(current_grasp_type)) {
    std::cerr << "Provided grasp type is not supported." << std::endl;
    this->cfg.grasp_type = current_grasp_type;
  }
  // this->add_collision_geoms(cfg.ignored_collision_geoms,
  //                           this->ignored_collision_geoms, true);
  return true;
}

bool SimTilburgHand::set_grasp_type(common::GraspType grasp_type){
  switch (grasp_type) {
    case common::GraspType::POWER_GRASP:
      this->cfg.grasp_type = common::GraspType::POWER_GRASP;
      break;
    default:
      return false;
  }
  return true;
}

SimTilburgHandConfig *SimTilburgHand::get_parameters() {
  // copy config to heap
  SimTilburgHandConfig *cfg = new SimTilburgHandConfig();
  *cfg = this->cfg;
  return cfg;
}

SimTilburgHandState *SimTilburgHand::get_state() {
  SimTilburgHandState *state = new SimTilburgHandState();
  *state = this->state;
  return state;
}
void SimTilburgHand::set_normalized_joint_poses(
    const rcs::common::VectorXd &q) {
  if (q.size() != this->n_joints) {
    throw std::invalid_argument("Invalid joint pose vector size, expected 16.");
  }
  this->state.last_commanded_qpos = q;
  for (int i = 0; i < this->n_joints; ++i) {
    this->sim->d->ctrl[this->actuator_ids[i]] =
        (mjtNum)q[i] * (this->cfg.max_joint_position[i] -
                        this->cfg.min_joint_position[i]) +
        this->cfg.min_joint_position[i];
  }

  // we ignore force for now
  // this->sim->d->actuator_force[this->gripper_id] = 0;
}
rcs::common::VectorXd SimTilburgHand::get_normalized_joint_poses() {
  // TODO: maybe we should use the mujoco sensors? Not sure what the difference
  // is between reading out from qpos and reading from the sensors.
  rcs::common::VectorXd q(this->n_joints);
  for (int i = 0; i < this->n_joints; ++i) {
    q[i] = (this->sim->d->qpos[this->joint_ids[i]] -
            this->cfg.min_joint_position[i]) /
           (this->cfg.max_joint_position[i] - this->cfg.min_joint_position[i]);
  }
  return q;
}

bool SimTilburgHand::collision_callback() {
  this->state.collision = false;
  // for (size_t i = 0; i < this->sim->d->ncon; ++i) {
  //   if (this->cfgeom.contains(this->sim->d->contact[i].geom[0]) &&
  //       this->cfgeom.contains(this->sim->d->contact[i].geom[1])) {
  //     // ignore collisions between fingers
  //     continue;
  //   }

  //   if ((this->cgeom.contains(this->sim->d->contact[i].geom[0]) or
  //        this->cgeom.contains(this->sim->d->contact[i].geom[1])) and
  //       // ignore all collision with ignored objects with frankahand
  //       // not just fingers
  //       not(this->ignored_collision_geoms.contains(
  //               this->sim->d->contact[i].geom[1]) or
  //           this->ignored_collision_geoms.contains(
  //               this->sim->d->contact[i].geom[1]))) {
  //     this->state.collision = true;
  //     break;
  //   }
  // }
  return this->state.collision;
}

bool SimTilburgHand::is_grasped() {
  // double width = this->get_normalized_width();

  // if (this->state.last_commanded_width - this->cfg.epsilon_inner < width &&
  //     width < this->state.last_commanded_width + this->cfg.epsilon_outer) {
  //   // this is the libfranka logic to decide whether an object is grasped
  //   return true;
  // }
  return false;
}

bool SimTilburgHand::convergence_callback() {
  auto qpos = get_normalized_joint_poses();
  this->state.is_moving =
      (this->state.last_qpos - qpos).norm() >
      0.001 * (this->cfg.max_joint_position - this->cfg.min_joint_position)
                  .norm();  // 0.1% tolerance
  this->state.last_qpos = qpos;
  return not this->state.is_moving;
}

void SimTilburgHand::grasp() {
  switch (this->cfg.grasp_type) {
    case common::GraspType::POWER_GRASP:
      this->set_normalized_joint_poses(this->power_grasp_pose);
      break;
    default:
      std::cerr << "Grasp type not implemented, using power grasp as default."
                << std::endl;
      this->set_normalized_joint_poses(this->power_grasp_pose);
      break;
  }
}

void SimTilburgHand::open() {
  this->set_normalized_joint_poses(this->open_pose);
}
void SimTilburgHand::shut() {
  this->set_normalized_joint_poses(rcs::common::VectorXd::Ones(16));
}

void SimTilburgHand::m_reset() {
  this->state = SimTilburgHandState();
  // reset state hard
  for (int i = 0; i < this->n_joints; ++i) {
    this->sim->d->qpos[this->joint_ids[i]] = this->cfg.min_joint_position[i];
    this->sim->d->ctrl[this->actuator_ids[i]] = this->cfg.min_joint_position[i];
  }
}

void SimTilburgHand::reset() { this->m_reset(); }
}  // namespace sim
}  // namespace rcs
