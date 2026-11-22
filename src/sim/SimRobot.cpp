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

#include "mujoco/mjdata.h"
#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rcs/Robot.h"

namespace rcs {
namespace sim {

// TODO: check dof contraints
// TODO: use C++11 feature to call one constructor from another
SimRobot::SimRobot(std::shared_ptr<Sim> sim,
                   std::shared_ptr<common::Kinematics> ik, SimRobotConfig cfg,
                   bool register_convergence_callback)
    : sim{sim}, cfg{cfg}, state{}, m_ik(ik) {
  this->init_ids();
  this->set_config(cfg);
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
  if (this->cfg.q_home.has_value()) {
    this->set_joint_position(this->cfg.q_home.value());
  } else {
    std::cerr
        << "Warning: No home position defined for robot, cannot move home."
        << std::endl;
  }
}

void SimRobot::init_ids() {
  std::string name;
  // Collision geoms
  for (size_t i = 0; i < std::size(this->cfg.arm_collision_geoms); ++i) {
    name = this->cfg.arm_collision_geoms[i];
    int id = mj_name2id(this->sim->m, mjOBJ_GEOM, name.c_str());
    if (id == -1) {
      throw std::runtime_error(std::string("No geom named " + name));
    }
    this->ids.cgeom.insert(id);
  }
  // Attachment Site
  name = this->cfg.attachment_site;
  this->ids.attachment_site =
      mj_name2id(this->sim->m, mjOBJ_SITE, name.c_str());
  if (this->ids.attachment_site == -1) {
    throw std::runtime_error(std::string("No site named " + name));
  }
  // Base
  name = this->cfg.base;
  this->ids.base = mj_name2id(this->sim->m, mjOBJ_BODY, name.c_str());
  if (this->ids.base == -1) {
    throw std::runtime_error(std::string("No body named " + name));
  }
  // Joints
  for (size_t i = 0; i < std::size(this->cfg.joints); ++i) {
    name = this->cfg.joints[i];
    this->ids.joints.push_back(
        mj_name2id(this->sim->m, mjOBJ_JOINT, name.c_str()));
    if (this->ids.joints[i] == -1) {
      throw std::runtime_error(std::string("No joint named " + name));
    }
  }
  // Actuators
  for (size_t i = 0; i < std::size(this->cfg.actuators); ++i) {
    name = this->cfg.actuators[i];
    this->ids.actuators.push_back(
        mj_name2id(this->sim->m, mjOBJ_ACTUATOR, name.c_str()));
    if (this->ids.actuators[i] == -1) {
      throw std::runtime_error(std::string("No actuator named " + name));
    }
  }
}

bool SimRobot::set_config(const SimRobotConfig& cfg) {
  common::check_robot_config(cfg);
  if (this->cfg.controller.has_value() && this->torque_control &&
      !cfg.controller.has_value()) {
    throw std::runtime_error("cannot switch back from torque control");
  }
  this->cfg = cfg;
  this->state.inverse_tcp_offset = cfg.tcp_offset.inverse();
  if (this->dynamics) {
    this->configure_dynamics();
    if (cfg.controller.has_value()) {
      this->install_configured_controller();
    }
  }
  if (this->torque_control && cfg.kp.has_value()) {
    throw std::runtime_error("kp/kv have no effect under torque control");
  }
  if (cfg.kp.has_value() != cfg.kv.has_value())
    throw std::runtime_error("kp and kv must both be set or unset");
  if (cfg.kp.has_value()) {
    size_t n = std::size(this->ids.actuators);
    if (cfg.kp->size() != n || cfg.kv->size() != n)
      throw std::runtime_error("kp/kv size must match number of joints");
    for (size_t i = 0; i < n; ++i) {
      int act = this->ids.actuators[i];
      if (this->sim->m->actuator_gaintype[act] != mjGAIN_FIXED ||
          this->sim->m->actuator_biastype[act] != mjBIAS_AFFINE)
        throw std::runtime_error("kp/kv require a position actuator");
      this->sim->m->actuator_gainprm[act * mjNGAIN + 0] = (*cfg.kp)[i];
      this->sim->m->actuator_biasprm[act * mjNBIAS + 1] = -(*cfg.kp)[i];
      this->sim->m->actuator_biasprm[act * mjNBIAS + 2] = -(*cfg.kv)[i];
    }
  }
  return true;
}

SimRobotConfig* SimRobot::get_config() {
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
  return this->get_cartesian_flange_position() * cfg.tcp_offset;
}

common::Pose SimRobot::get_cartesian_flange_position() {
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation(
      this->sim->d->site_xmat + 9 * this->ids.attachment_site);
  Eigen::Vector3d translation(this->sim->d->site_xpos +
                              3 * this->ids.attachment_site);
  common::Pose attachment_site(Eigen::Matrix3d(rotation), translation);
  return this->to_pose_in_robot_coordinates(attachment_site);
}

void SimRobot::set_joint_position(const common::VectorXd& q) {
  if (this->torque_control && !this->joint_controller) {
    // only a Cartesian controller: move the TCP to where q would put it
    this->set_cartesian_position(
        this->dynamics->compute(q, common::VectorXd::Zero(q.size()), 0.0)
            .tcp_pose);
    return;
  }
  this->state.target_angles = q;
  this->state.previous_angles = this->get_joint_position();
  this->state.is_moving = true;
  this->state.is_arrived = false;
  if (this->torque_control) {
    if (this->control_mode != ControlMode::joint) {
      this->activate(ControlMode::joint);
    }
    this->joint_controller->set_goal(q);
    return;
  }
  for (size_t i = 0; i < std::size(this->ids.actuators); ++i) {
    this->sim->d->ctrl[this->ids.actuators[i]] = q[i];
  }
}

common::VectorXd SimRobot::get_joint_position() {
  return m_get_joint_position();
}

common::VectorXd SimRobot::m_get_joint_position() {
  common::VectorXd q(std::size(this->cfg.joints));
  for (size_t i = 0; i < std::size(this->cfg.joints); ++i) {
    q[i] = this->sim->d->qpos[this->sim->m->jnt_qposadr[this->ids.joints[i]]];
  }
  return q;
}

std::optional<std::shared_ptr<common::Kinematics>> SimRobot::get_ik() {
  return this->m_ik;
}

void SimRobot::set_cartesian_position(const common::Pose& pose) {
  // pose is assumed to be in the robots coordinate frame
  if (this->cartesian_controller) {
    this->state.target_pose = pose;
    this->state.previous_angles = this->get_joint_position();
    this->state.ik_success = true;
    this->state.is_moving = true;
    this->state.is_arrived = false;
    if (this->control_mode != ControlMode::cartesian) {
      this->activate(ControlMode::cartesian);
    }
    this->cartesian_controller->set_goal(pose);
    return;
  }
  auto joint_vals = this->m_ik->inverse(pose, this->get_joint_position(),
                                        this->cfg.tcp_offset);
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
  if (this->control_mode == ControlMode::cartesian) {
    // TCP within 1 mm and 1 degree of the goal
    this->state.is_arrived = this->get_cartesian_position().is_close(
        this->state.target_pose, std::numbers::pi / 180.0, 1e-3);
    return;
  }
  common::VectorXd current_angles = this->get_joint_position();
  this->state.is_arrived =
      (current_angles - this->state.target_angles).cwiseAbs().maxCoeff() <
      this->cfg.joint_rotational_tolerance;
}

bool SimRobot::collision_callback() {
  for (size_t i = 0; i < this->sim->d->ncon; ++i) {
    if (this->ids.cgeom.contains(this->sim->d->contact[i].geom[0]) ||
        this->ids.cgeom.contains(this->sim->d->contact[i].geom[1])) {
      this->state.collision = true;
      break;
    }
  }
  return this->state.collision;
}

void SimRobot::clear_collision_flag() { this->state.collision = false; }

bool SimRobot::convergence_callback() {
  /* When ik failed, the robot is not doing anything */
  if (not this->state.ik_success) {
    return true;
  }
  /* Otherwise we are done when we arrived and stopped moving */
  return this->state.is_arrived and not this->state.is_moving;
}

void SimRobot::m_reset() {
  this->state = SimRobotState();
  // if has qhome
  if (this->cfg.q_home.has_value()) {
    this->set_joints_hard(this->cfg.q_home.value());
  }
  this->state.previous_angles = this->m_get_joint_position();
  this->state.target_angles = this->m_get_joint_position();
  this->state.is_arrived = true;
  this->state.is_moving = false;
}

void SimRobot::set_joints_hard(const common::VectorXd& q) {
  // TODO does not work for some reason
  this->state.target_angles = q;
  this->state.previous_angles = this->get_joint_position();
  for (size_t i = 0; i < std::size(this->ids.joints); ++i) {
    size_t jnt_id = this->ids.joints[i];
    size_t jnt_qposadr = this->sim->m->jnt_qposadr[jnt_id];
    this->sim->d->qpos[jnt_qposadr] = q[i];
    if (!this->torque_control) {
      this->sim->d->ctrl[this->ids.actuators[i]] = q[i];
    }
  }
  if (this->torque_control) {
    // hold the new configuration instead of the old setpoint
    this->activate(this->control_mode);
    this->control_step(0.0);
  }
}

common::Pose SimRobot::get_base_pose_in_world_coordinates() {
  Eigen::Map<Eigen::Vector3d> translation(this->sim->d->xpos +
                                          3 * this->ids.base);
  auto quat = this->sim->d->xquat + 4 * this->ids.base;
  Eigen::Quaterniond rotation(quat[0], quat[1], quat[2], quat[3]);
  return common::Pose(rotation, translation);
}

void SimRobot::reset() { this->m_reset(); }

void SimRobot::set_dynamics(
    std::shared_ptr<common::PinocchioDynamics> dynamics) {
  if (dynamics->dof() != std::size(this->ids.joints)) {
    throw std::invalid_argument(
        "dynamics model and robot have a different number of joints");
  }
  this->dynamics = dynamics;
  this->configure_dynamics();
  if (this->cfg.controller.has_value()) {
    this->install_configured_controller();
  }
}

void SimRobot::install_configured_controller() {
  SimConfig sim_cfg = this->sim->get_config();
  if (!sim_cfg.async_control) {
    // synchronous control steps until the goal is reached, which friction
    // can prevent under torque control
    throw std::runtime_error(
        "a configured controller needs SimConfig::async_control");
  }
  common::ControllerPair controller =
      common::make_controller(this->cfg.controller.value(),
                              1.0 / sim_cfg.frequency, this->cfg.joint_limits);
  this->joint_controller = nullptr;
  this->cartesian_controller = nullptr;
  if (controller.joint) {
    this->set_joint_controller(controller.joint);
  } else {
    this->set_cartesian_controller(controller.cartesian);
  }
}

void SimRobot::configure_dynamics() {
  this->dynamics->set_tcp_offset(this->cfg.tcp_offset);
  common::Payload simulated = this->get_simulated_payload();  // runs kinematics
  // gravity in the robot base frame, for robots that are mounted tilted
  Eigen::Map<Eigen::Vector3d> gravity_world(this->sim->m->opt.gravity);
  this->dynamics->set_gravity(
      this->get_base_pose_in_world_coordinates().rotation_m().transpose() *
      gravity_world);
  if (!this->cfg.payload.has_value()) {
    this->dynamics->set_payload(simulated);
    return;
  }
  const common::Payload& configured = this->cfg.payload.value();
  this->dynamics->set_payload(configured);
  // a gap acts on the arm like a wrong tool configuration on the real robot
  if (std::abs(configured.mass - simulated.mass) > 0.05 ||
      (configured.com - simulated.com).norm() > 0.01) {
    std::cerr << "Warning: the configured payload (" << configured.mass
              << " kg, CoM " << configured.com.transpose()
              << ") differs from the simulated bodies after the flange ("
              << simulated.mass << " kg, CoM " << simulated.com.transpose()
              << ")." << std::endl;
  }
}

common::Payload SimRobot::get_simulated_payload() {
  mjModel* m = this->sim->m;
  mjData* d = this->sim->d;
  mj_kinematics(m, d);
  int flange_body = m->site_bodyid[this->ids.attachment_site];
  // all bodies in the subtree below the flange body
  double mass = 0.0;
  Eigen::Vector3d moment = Eigen::Vector3d::Zero();
  std::vector<int> bodies;
  for (int b = flange_body + 1; b < m->nbody; ++b) {
    int parent = m->body_parentid[b];
    while (parent != 0 && parent != flange_body) {
      parent = m->body_parentid[parent];
    }
    if (parent == flange_body) {
      bodies.push_back(b);
      mass += m->body_mass[b];
      moment += m->body_mass[b] * Eigen::Map<Eigen::Vector3d>(d->xipos + 3 * b);
    }
  }
  common::Payload payload;
  if (mass <= 0.0) {
    return payload;
  }
  Eigen::Vector3d com = moment / mass;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();  // about com, world axes
  for (int b : bodies) {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R(d->ximat + 9 * b);
    Eigen::Vector3d r = Eigen::Map<Eigen::Vector3d>(d->xipos + 3 * b) - com;
    inertia +=
        R * Eigen::Map<Eigen::Vector3d>(m->body_inertia + 3 * b).asDiagonal() *
            R.transpose() +
        m->body_mass[b] *
            (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose());
  }
  // express in the flange frame
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> R_flange(
      d->site_xmat + 9 * this->ids.attachment_site);
  Eigen::Vector3d p_flange(d->site_xpos + 3 * this->ids.attachment_site);
  payload.mass = mass;
  payload.com = R_flange.transpose() * (com - p_flange);
  payload.inertia = R_flange.transpose() * inertia * R_flange;
  return payload;
}

void SimRobot::set_joint_controller(
    std::shared_ptr<common::JointController> controller) {
  this->joint_controller = controller;
  this->enable_torque_control();
  this->activate(ControlMode::joint);
  this->control_step(0.0);
}

void SimRobot::set_cartesian_controller(
    std::shared_ptr<common::CartesianController> controller) {
  this->cartesian_controller = controller;
  this->enable_torque_control();
  this->activate(ControlMode::cartesian);
  this->state.target_pose = this->get_cartesian_position();
  this->control_step(0.0);
}

void SimRobot::enable_torque_control() {
  if (!this->dynamics) {
    throw std::logic_error(
        "call set_dynamics() before installing a controller");
  }
  if (this->torque_control) {
    return;
  }
  mjModel* m = this->sim->m;
  // actuators: position servos -> motors, limited to the joint torque range
  for (size_t i = 0; i < std::size(this->ids.actuators); ++i) {
    int act = this->ids.actuators[i];
    int jnt = this->ids.joints[i];
    m->actuator_gaintype[act] = mjGAIN_FIXED;
    m->actuator_biastype[act] = mjBIAS_NONE;
    std::fill_n(m->actuator_gainprm + act * mjNGAIN, mjNGAIN, 0.0);
    std::fill_n(m->actuator_biasprm + act * mjNBIAS, mjNBIAS, 0.0);
    m->actuator_gainprm[act * mjNGAIN] = 1.0;
    m->actuator_ctrllimited[act] = m->jnt_actfrclimited[jnt];
    m->actuator_ctrlrange[2 * act] = m->jnt_actfrcrange[2 * jnt];
    m->actuator_ctrlrange[2 * act + 1] = m->jnt_actfrcrange[2 * jnt + 1];
    m->jnt_actgravcomp[jnt] = 0;
  }
  // gravity is compensated from the configured payload, as by the firmware
  size_t compensated = 0;
  for (int b = this->ids.base; b < m->nbody; ++b) {
    int parent = b;
    while (parent != 0 && parent != this->ids.base) {
      parent = m->body_parentid[parent];
    }
    if (parent == this->ids.base && m->body_gravcomp[b] != 0) {
      m->body_gravcomp[b] = 0;
      compensated++;
    }
  }
  if (compensated > 0) {
    std::cerr << "Torque control: switched off MuJoCo gravity compensation of "
              << compensated << " bodies of robot " << this->cfg.base
              << std::endl;
  }
  this->torque_control = true;
  // after every physics step; on a restart (reset, time going backwards)
  // hold the configuration the robot is in now
  this->sim->register_cb(
      [this]() { this->control_step(this->sim->m->opt.timestep); }, 0.0,
      [this]() {
        this->activate(this->control_mode);
        this->control_step(0.0);
      });
}

void SimRobot::activate(ControlMode mode) {
  common::ControllerState state = this->controller_state(0.0);
  if (mode == ControlMode::joint) {
    this->joint_controller->reset(state);
  } else if (mode == ControlMode::cartesian) {
    this->cartesian_controller->reset(state);
  }
  this->control_mode = mode;
}

common::ControllerState SimRobot::controller_state(double dt) {
  mjModel* m = this->sim->m;
  mjData* d = this->sim->d;
  size_t n = std::size(this->ids.joints);
  common::VectorXd q(n), dq(n);
  for (size_t i = 0; i < n; ++i) {
    q[i] = d->qpos[m->jnt_qposadr[this->ids.joints[i]]];
    dq[i] = d->qvel[m->jnt_dofadr[this->ids.joints[i]]];
  }
  return this->dynamics->compute(q, dq, dt);
}

void SimRobot::control_step(double dt) {
  if (this->control_mode == ControlMode::native) {
    return;
  }
  common::ControllerState state = this->controller_state(dt);
  common::VectorXd tau = this->control_mode == ControlMode::joint
                             ? this->joint_controller->step(state)
                             : this->cartesian_controller->step(state);
  tau += state.gravity;  // the firmware's part
  for (size_t i = 0; i < std::size(this->ids.actuators); ++i) {
    this->sim->d->ctrl[this->ids.actuators[i]] = tau[i];
  }
}

}  // namespace sim
}  // namespace rcs
