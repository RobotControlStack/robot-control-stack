#ifndef RCS_ROBOT_H
#define RCS_ROBOT_H

#include <memory>
#include <optional>
#include <string>

#include "Pose.h"

namespace rcs {
namespace common {

typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
typedef Eigen::Matrix<int, 7, 1, Eigen::ColMajor> Vector7i;

struct RConfig {
  virtual ~RConfig(){};
};
struct RState {
  virtual ~RState(){};
};

struct GConfig {
  virtual ~GConfig(){};
};
struct GState {
  virtual ~GState(){};
};

class Robot {
 public:
  virtual ~Robot(){};

  // Also add an implementation specific set_parameters function that takes
  // a deduced config type
  // This functino can also be used for signaling e.g. error recovery and the
  // like
  // bool set_parameters(const RConfig& cfg);

  virtual RConfig* get_parameters() = 0;

  virtual RState* get_state() = 0;

  virtual Pose get_cartesian_position() = 0;

  virtual void set_joint_position(const Vector7d& q) = 0;

  virtual Vector7d get_joint_position() = 0;

  virtual void move_home() = 0;

  virtual void set_cartesian_position(const Pose& pose) = 0;
};

class Gripper {
 public:
  virtual ~Gripper(){};

  // Also add an implementation specific set_parameters function that takes
  // a deduced config type
  // bool set_parameters(const GConfig& cfg);

  virtual GConfig* get_parameters() = 0;
  virtual GState* get_state() = 0;

  // close gripper with force, return true if the object is grasped successfully
  virtual bool grasp() = 0;

  // open gripper
  virtual void release() = 0;

  // close gripper without applying force
  virtual void shut() = 0;
};

class RobotWithGripper {
 public:
  std::shared_ptr<Robot> robot;
  std::optional<std::shared_ptr<Gripper>> gripper;

  RobotWithGripper(std::shared_ptr<Robot> robot,
                   std::optional<std::shared_ptr<Gripper>> gripper)
      : robot(robot), gripper(gripper) {}

  ~RobotWithGripper() {}
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_ROBOT_H