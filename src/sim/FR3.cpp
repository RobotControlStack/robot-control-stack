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
  std::vector<std::string> arm = {"fr3_link0_collision", "fr3_link1_collision",
                                  "fr3_link2_collision", "fr3_link3_collision",
                                  "fr3_link4_collision", "fr3_link5_collision",
                                  "fr3_link6_collision", "fr3_link7_collision"};
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
  std::array<std::string, 7> joints = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
  std::array<std::string, 7> actuators = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7",
  };
} const model_names;

static void init_geom_ids(std::set<size_t>& ids,
                          std::vector<std::string> const& names,
                          mjModel const& m, const std::string& id) {
  for (auto const& name : names) {
    std::string name_with_id = name;
    name_with_id.append("_");
    name_with_id.append(id);
    int id = mj_name2id(&m, mjOBJ_GEOM, name_with_id.c_str());
    if (id == -1)
      throw std::runtime_error(std::string("No geom named ") + name_with_id);
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

FR3::FR3(Sim sim, std::string& id, std::shared_ptr<rl::mdl::Model> rlmdl)
    : sim{sim},
      id{id},
      rl{.mdl = rlmdl,
         .kin = std::dynamic_pointer_cast<rl::mdl::Kinematic>(rlmdl)},
      cfg{},
      state{} {
  this->rl.ik =
      std::make_shared<rl::mdl::JacobianInverseKinematics>(this->rl.kin.get());
  this->rl.ik->setDuration(std::chrono::milliseconds(this->cfg.ik_duration));
  this->init_ids();
  this->reset();
}

void FR3::init_ids() {
  // Collision geoms
  init_geom_ids(this->ids.cgeom.arm, model_names.arm, *this->sim.m, this->id);
  init_geom_ids(this->ids.cgeom.hand, model_names.hand, *this->sim.m, this->id);
  init_geom_ids(this->ids.cgeom.gripper, model_names.gripper, *this->sim.m,
                this->id);
  // Sites
  std::string name = "attachment_site_";
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
  this->rl.ik->setDuration(std::chrono::milliseconds(this->cfg.ik_duration));
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
  for (int i = 0; i < std::size(model_names.joints); ++i) {
    this->sim.d->ctrl[this->sim.m->jnt_qposadr[this->ids.joints[i]]] = q[i];
  }
}

common::Vector7d FR3::get_joint_position() {
  common::Vector7d q;
  for (int i = 0; i < std::size(model_names.joints); ++i) {
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
}  // namespace sim
}  // namespace rcs
