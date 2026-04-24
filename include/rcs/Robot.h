#ifndef RCS_ROBOT_H
#define RCS_ROBOT_H

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "Kinematics.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

enum RobotPlatform { SIMULATION = 0, HARDWARE };

template <typename Derived>
struct TypeBase {
  std::string id;

  // Each subclass gets its own unique static registry of type ids.
  static std::set<std::string>& registry() {
    static std::set<std::string> _registry;
    return _registry;
  }

  // Constructor automatically registers the instance
  TypeBase(std::string id_) : id(std::move(id_)) { registry().insert(id); }

  // Returns all registered instances of this specific type
  static std::vector<Derived> get_all() {
    std::vector<Derived> all;
    all.reserve(registry().size());
    for (const auto& id : registry()) {
      all.emplace_back(id);
    }
    return all;
  }

  bool operator==(const TypeBase& other) const { return id == other.id; }
};
struct RobotType : public TypeBase<RobotType> {
  using TypeBase::TypeBase;  // Inherit the constructor

  static const RobotType FR3;
  static const RobotType Panda;
};
inline const RobotType RobotType::FR3{"FR3"};
inline const RobotType RobotType::Panda{"Panda"};

struct RobotConfig {
  RobotType robot_type = RobotType::FR3;
  RobotPlatform robot_platform = RobotPlatform::SIMULATION;
  rcs::common::Pose tcp_offset = rcs::common::Pose::Identity();
  std::string attachment_site = "attachment_site";
  std::string kinematic_model_path = "assets/scenes/fr3_empty_world/robot.xml";
  std::optional<VectorXd> q_home = std::nullopt;
  size_t dof = 7;
  Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor> joint_limits =
      (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
           // low 7‐tuple
           -2.3093,
       -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895,
       // high 7‐tuple
       2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895)
          .finished();
  virtual ~RobotConfig() {};
};
struct RobotState {
  virtual ~RobotState() {};
};

struct GripperType : public TypeBase<GripperType> {
  using TypeBase::TypeBase;

  static const GripperType FrankaHand;
};
inline const GripperType GripperType::FrankaHand{"FrankaHand"};

struct GripperConfig {
  GripperType gripper_type = GripperType::FrankaHand;
  virtual ~GripperConfig() {};
};
struct GripperState {
  virtual ~GripperState() {};
};

enum GraspType {
  POWER_GRASP = 0,
  PRECISION_GRASP,
  LATERAL_GRASP,
  TRIPOD_GRASP
};
struct HandConfig {
  virtual ~HandConfig() {};
};
struct HandState {
  virtual ~HandState() {};
};

class Robot {
 public:
  virtual ~Robot() {};

  // Also add an implementation specific set_config function that takes
  // a deduced config type
  // This function can also be used for signaling e.g. error recovery and the
  // like
  // bool set_config(const RConfig& cfg);

  virtual RobotConfig* get_config() = 0;

  virtual RobotState* get_state() = 0;

  virtual Pose get_cartesian_position() = 0;

  virtual void set_joint_position(const VectorXd& q) = 0;

  virtual VectorXd get_joint_position() = 0;

  virtual void move_home() = 0;

  virtual void reset() = 0;

  virtual void close() = 0;

  virtual void set_cartesian_position(const Pose& pose) = 0;

  virtual std::optional<std::shared_ptr<Kinematics>> get_ik() = 0;

  common::Pose to_pose_in_world_coordinates(
      const Pose& pose_in_robot_coordinates);

  common::Pose to_pose_in_robot_coordinates(
      const Pose& pose_in_world_coordinates);

  virtual common::Pose get_base_pose_in_world_coordinates() = 0;
};

class Gripper {
 public:
  virtual ~Gripper() {};

  // Also add an implementation specific set_config function that takes
  // a deduced config type
  // bool set_config(const GConfig& cfg);

  virtual GripperConfig* get_config() = 0;
  virtual GripperState* get_state() = 0;

  // set width of the gripper, 0 is closed, 1 is open
  virtual void set_normalized_width(double width, double force = 0) = 0;
  virtual double get_normalized_width() = 0;

  virtual bool is_grasped() = 0;

  // close gripper with force, return true if the object is grasped successfully
  virtual void grasp() = 0;

  // open gripper
  virtual void open() = 0;

  // close gripper without applying force
  virtual void shut() = 0;

  // puts the gripper to max position
  virtual void reset() = 0;

  // closes connection to gripper
  virtual void close() = 0;
};

class Hand {
 public:
  virtual ~Hand() {};
  // TODO: Add low-level control interface for the hand with internal state
  // updates Also add an implementation specific set_config function that takes
  // a deduced config type
  // bool set_config(const GConfig& cfg);

  virtual HandConfig* get_config() = 0;
  virtual HandState* get_state() = 0;

  // set width of the hand, 0 is closed, 1 is open
  virtual void set_normalized_joint_poses(const VectorXd& q) = 0;
  virtual VectorXd get_normalized_joint_poses() = 0;

  virtual bool is_grasped() = 0;

  // close hand with force, return true if the object is grasped successfully
  virtual void grasp() = 0;

  // open hand
  virtual void open() = 0;

  // close hand without applying force
  virtual void shut() = 0;

  // puts the hand to max position
  virtual void reset() = 0;

  // closes connection
  virtual void close() = 0;
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_ROBOT_H
