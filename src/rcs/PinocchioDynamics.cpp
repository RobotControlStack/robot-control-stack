#include "rcs/PinocchioDynamics.h"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>

namespace rcs {
namespace common {

PinocchioDynamics::PinocchioDynamics(const std::string& model_path, bool urdf,
                                     const std::string& flange_frame,
                                     const Pose& tcp_offset) {
  if (urdf) {
    pinocchio::urdf::buildModel(model_path, this->model);
  } else {
    pinocchio::mjcf::buildModel(model_path, this->model);
  }
  if (!this->model.existFrame(flange_frame)) {
    throw std::runtime_error("frame " + flange_frame + " not found in " +
                             model_path);
  }
  // franka::Model has no rotor inertia either
  this->model.armature.setZero();

  const pinocchio::Frame& flange =
      this->model.frames[this->model.getFrameId(flange_frame)];
  this->flange_joint = flange.parentJoint;
  this->flange_placement = flange.placement;
  this->arm_inertia = this->model.inertias[this->flange_joint];

  this->tcp_frame = this->model.addFrame(
      pinocchio::Frame("rcs_tcp", this->flange_joint, this->flange_placement,
                       pinocchio::OP_FRAME));
  this->data = pinocchio::Data(this->model);
  this->set_tcp_offset(tcp_offset);
}

void PinocchioDynamics::set_payload(const Payload& payload) {
  this->payload = payload;
  // add the payload to the body that carries the flange, as libfranka does
  pinocchio::Inertia load(payload.mass, payload.com, payload.inertia);
  this->model.inertias[this->flange_joint] =
      this->arm_inertia + this->flange_placement.act(load);
}

Payload PinocchioDynamics::get_payload() const { return this->payload; }

void PinocchioDynamics::set_tcp_offset(const Pose& tcp_offset) {
  pinocchio::SE3 offset(tcp_offset.rotation_m(), tcp_offset.translation());
  this->model.frames[this->tcp_frame].placement =
      this->flange_placement * offset;
}

void PinocchioDynamics::set_gravity(const Eigen::Vector3d& gravity) {
  this->model.gravity.linear(gravity);
}

size_t PinocchioDynamics::dof() const { return this->model.nv; }

ControllerState PinocchioDynamics::compute(const VectorXd& q,
                                           const VectorXd& dq, double dt) {
  if (q.size() != this->model.nq || dq.size() != this->model.nv) {
    throw std::invalid_argument("q and dq must have " +
                                std::to_string(this->model.nv) + " entries");
  }
  ControllerState state;
  state.q = q;
  state.dq = dq;
  state.dt = dt;

  // crba fills only the upper triangle
  pinocchio::crba(this->model, this->data, q);
  this->data.M.triangularView<Eigen::StrictlyLower>() =
      this->data.M.transpose().triangularView<Eigen::StrictlyLower>();
  state.mass = this->data.M;

  state.gravity =
      pinocchio::computeGeneralizedGravity(this->model, this->data, q);
  // rnea with zero acceleration gives Coriolis + gravity
  VectorXd zero = VectorXd::Zero(this->model.nv);
  state.coriolis =
      pinocchio::rnea(this->model, this->data, q, dq, zero) - state.gravity;

  pinocchio::framesForwardKinematics(this->model, this->data, q);
  const pinocchio::SE3& tcp = this->data.oMf[this->tcp_frame];
  state.tcp_pose =
      Pose(Eigen::Matrix3d(tcp.rotation()), Eigen::Vector3d(tcp.translation()));

  state.jacobian.setZero(6, this->model.nv);
  pinocchio::computeFrameJacobian(this->model, this->data, q, this->tcp_frame,
                                  pinocchio::LOCAL_WORLD_ALIGNED,
                                  state.jacobian);
  return state;
}

}  // namespace common
}  // namespace rcs
