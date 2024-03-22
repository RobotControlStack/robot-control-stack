#ifndef RCS_ROBOT_H
#define RCS_ROBOT_H

#include <pybind11/pybind11.h>
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

  // set_parameters could also be used for signaling e.g. error recovery and the
  // like
  virtual bool set_parameters(const RConfig &cfg) = 0;
  virtual std::unique_ptr<RConfig> get_parameters() = 0;

  virtual std::unique_ptr<RState> get_state() = 0;

  virtual Pose get_cartesian_position() = 0;

  virtual void set_joint_position(const Vector7d &q) = 0;

  virtual Vector7d get_joint_position() = 0;

  virtual void move_home() = 0;

  virtual void set_cartesian_position(const Pose &pose) = 0;
};

class Gripper {
 public:
  virtual ~Gripper(){};

  virtual bool set_parameters(const GConfig &cfg) = 0;
  virtual std::unique_ptr<GConfig> get_parameters() = 0;
  virtual std::unique_ptr<GState> get_state() = 0;

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