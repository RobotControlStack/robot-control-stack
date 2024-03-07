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

template <class RobotBase = Robot>
class PyRobot : public RobotBase {
 public:
  using RobotBase::RobotBase;  // Inherit constructors

  void set_parameters(const RConfig &cfg) override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, set_parameters, cfg);
  }

  RConfig get_parameters() override {
    PYBIND11_OVERRIDE_PURE(RConfig, RobotBase, get_parameters, );
  }

  RState get_state() override {
    PYBIND11_OVERRIDE_PURE(RState, RobotBase, get_state, );
  }

  Pose get_cartesian_position() override {
    PYBIND11_OVERRIDE_PURE(Pose, RobotBase, get_cartesian_position, );
  }

  void set_joint_position(const Vector7d &q) override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, set_joint_position, q);
  }

  Vector7d get_joint_position() override {
    PYBIND11_OVERRIDE_PURE(Vector7d, RobotBase, get_joint_position, );
  }

  void move_home() override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, move_home, );
  }

  void set_cartesian_position(const Pose &pose) override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, set_cartesian_position, pose);
  }
};

class Gripper {
 public:
  Gripper(){};
  virtual ~Gripper(){};

  virtual void set_parameters(const GConfig &cfg) = 0;
  virtual GConfig get_parameters() = 0;
  virtual GState get_state() = 0;

  // close gripper with force, return true if the object is grasped successfully
  virtual bool grasp() = 0;

  // open gripper
  virtual void release() = 0;

  // close gripper without applying force
  virtual void shut() = 0;
};

template <class GripperBase = Gripper>
class PyGripper : public GripperBase {
 public:
  using GripperBase::GripperBase;  // Inherit constructors

  void set_parameters(const GConfig &cfg) override {
    PYBIND11_OVERRIDE_PURE(void, GripperBase, set_parameters, cfg);
  }

  GConfig get_parameters() override {
    PYBIND11_OVERRIDE_PURE(GConfig, GripperBase, get_parameters, );
  }

  GState get_state() override {
    PYBIND11_OVERRIDE_PURE(GState, GripperBase, get_state, );
  }

  bool grasp() override { PYBIND11_OVERRIDE_PURE(bool, GripperBase, grasp, ); }

  void release() override {
    PYBIND11_OVERRIDE_PURE(void, GripperBase, release, );
  }

  void shut() override { PYBIND11_OVERRIDE_PURE(void, GripperBase, shut, ); }
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