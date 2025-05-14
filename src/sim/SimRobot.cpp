#include "SimRobot.h"

#include <Eigen/src/Core/Matrix.h>
#include <math.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "common/Robot.h"
#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"

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

// TODO: use C++11 feature to call one constructor from another
SimRobot::SimRobot(std::shared_ptr<Sim> sim, const std::string& id,
                   std::shared_ptr<common::IK> ik,
                   bool register_convergence_callback)
    : sim{sim}, id{id}, cfg{}, state{}, m_ik(ik) {
  this->init_ids();
  if (register_convergence_callback) {
    this->sim->register_cb(std::bind(&SimRobot::is_arrived_callback, this),
                           this->cfg.seconds_between_callbacks);
    this->sim->register_cb(std::bind(&SimRobot::is_moving_callback, this),
                           this->cfg.seconds_between_callbacks);
    this->sim->register_all_cb(std::bind(&SimRobot::convergence_callback, this),
                               this->cfg.seconds_between_callbacks);
  }
  this->sim->register_any_cb(std::bind(&SimRobot::collision_callback, this),
                             this->cfg.seconds_between_callbacks);
  this->m_reset();
}

SimRobot::~SimRobot() {}

void SimRobot::move_home() {
  this->set_joint_position(
      common::robots_meta_config.at(this->cfg.robot_type).q_home);
}

void SimRobot::init_ids() {
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

bool SimRobot::set_parameters(const SimRobotConfig& cfg) {
  this->cfg = cfg;
  this->state.inverse_tcp_offset = cfg.tcp_offset.inverse();
  return true;
}

SimRobotConfig* SimRobot::get_parameters() {
  SimRobotConfig* cfg = new SimRobotConfig();
  *cfg = this->cfg;
  return cfg;
}

SimRobotState* SimRobot::get_state() {
  SimRobotState* state = new SimRobotState();
  *state = this->state;
  return state;
}

common::Pose SimRobot::get_cartesian_position() {
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(
      this->sim->d->site_xmat + 9 * this->ids.attachment_site);
  Eigen::Vector3d translation(this->sim->d->site_xpos +
                              3 * this->ids.attachment_site);
  common::Pose attachment_site(Eigen::Matrix3d(rotation), translation);
  return this->to_pose_in_robot_coordinates(attachment_site) * cfg.tcp_offset;
}

void SimRobot::set_joint_position(const common::VectorXd& q) {
  this->state.target_angles = q;
  this->state.previous_angles = this->get_joint_position();
  this->state.is_moving = true;
  this->state.is_arrived = false;
  for (size_t i = 0; i < std::size(this->ids.actuators); ++i) {
    this->sim->d->ctrl[this->ids.actuators[i]] = q[i];
  }
}

common::VectorXd SimRobot::get_joint_position() {
  common::VectorXd q(std::size(model_names.joints));
  for (size_t i = 0; i < std::size(model_names.joints); ++i) {
    q[i] = this->sim->d->qpos[this->sim->m->jnt_qposadr[this->ids.joints[i]]];
  }
  return q;
}

std::optional<std::shared_ptr<common::IK>> SimRobot::get_ik() {
  return this->m_ik;
}

void SimRobot::set_cartesian_position(const common::Pose& pose) {
  // pose is assumed to be in the robots coordinate frame
  auto joint_vals =
      this->m_ik->ik(pose, this->get_joint_position(), this->cfg.tcp_offset);
  if (joint_vals.has_value()) {
    this->state.ik_success = true;
    this->set_joint_position(joint_vals.value());
  } else {
    this->state.ik_success = false;
  }
}
void SimRobot::is_moving_callback() {
  common::VectorXd current_angles = this->get_joint_position();
  // difference of the largest element is smaller than threshold
  this->state.is_moving =
      (current_angles - this->state.previous_angles).cwiseAbs().maxCoeff() >
      0.0001;
  this->state.previous_angles = current_angles;
}

void SimRobot::is_arrived_callback() {
  common::VectorXd current_angles = this->get_joint_position();
  this->state.is_arrived =
      (current_angles - this->state.target_angles).cwiseAbs().maxCoeff() <
      this->cfg.joint_rotational_tolerance;
}

bool SimRobot::collision_callback() {
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

bool SimRobot::convergence_callback() {
  /* When ik failed, the robot is not doing anything */
  if (not this->state.ik_success) {
    return true;
  }
  /* Otherwise we are done when we arrived and stopped moving */
  return this->state.is_arrived and not this->state.is_moving;
}

void SimRobot::m_reset() {
  this->set_joints_hard(
      common::robots_meta_config.at(this->cfg.robot_type).q_home);
}

void SimRobot::set_joints_hard(const common::VectorXd& q) {
  for (size_t i = 0; i < std::size(this->ids.joints); ++i) {
    size_t jnt_id = this->ids.joints[i];
    size_t jnt_qposadr = this->sim->m->jnt_qposadr[jnt_id];
    this->sim->d->qpos[jnt_qposadr] = q[i];
    this->sim->d->ctrl[this->ids.actuators[i]] = q[i];
  }
}

common::Pose SimRobot::get_base_pose_in_world_coordinates() {
  auto id = mj_name2id(this->sim->m, mjOBJ_BODY,
                       (std::string("base_") + this->id).c_str());
  Eigen::Map<Eigen::Vector3d> translation(this->sim->d->xpos + 3 * id);
  auto quat = this->sim->d->xquat + 4 * id;
  Eigen::Quaterniond rotation(quat[0], quat[1], quat[2], quat[3]);
  return common::Pose(rotation, translation);
}

void SimRobot::reset() { this->m_reset(); }
}  // namespace sim
}  // namespace rcs
