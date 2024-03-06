#ifndef RCS_ROBOT_H
#define RCS_ROBOT_H

#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>

#include <memory>
#include <optional>
#include <string>

#include "Pose.h"

namespace rcs {
namespace common {

typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
typedef Eigen::Matrix<int, 7, 1, Eigen::ColMajor> Vector7i;

struct RConfig {};
struct RState {};

struct GConfig {};
struct GState {};

class Robot {
 public:
  Robot(){};
  virtual ~Robot(){};

  virtual void set_parameters(const RConfig &) = 0;
  virtual RConfig get_parameters() = 0;

  virtual RState get_state() = 0;

  virtual Pose get_cartesian_position() = 0;

  virtual void set_joint_position(const Vector7d &q) = 0;

  virtual Vector7d get_joint_position() = 0;

  virtual void move_home() = 0;

  virtual void set_cartesian_position(const Pose &pose) = 0;
};

class Gripper {
 public:
  Gripper(){};
  virtual ~Gripper(){};

  virtual void set_parameters(const GConfig &) = 0;
  virtual GConfig get_parameters() = 0;
  virtual GState get_state() = 0;

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