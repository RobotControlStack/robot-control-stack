#ifndef RCS_ROBOT_H
#define RCS_ROBOT_H

#include <memory>
#include <optional>
#include <string>

#include "IK.h"
#include "Pose.h"
#include "utils.h"

namespace rcs {
namespace common {

struct RobotMetaConfig {
  VectorXd q_home;
  int dof;
  Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor> joint_limits;
};

enum RobotType { FR3 = 0, UR5e, SO101, XArm7 };
enum RobotPlatform { SIMULATION = 0, HARDWARE };

static const std::unordered_map<RobotType, RobotMetaConfig> robots_meta_config =
    {{// -------------- FR3 --------------
      {FR3,
       RobotMetaConfig{
           // q_home:
           (VectorXd(7) << 0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2,
            M_PI_4)
               .finished(),
           // dof:
           7,
           // joint_limits:
           (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
                // low 7‐tuple
                -2.3093,
            -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895,
            // high 7‐tuple
            2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895)
               .finished()}},

      // -------------- UR5e --------------
      {UR5e,
       RobotMetaConfig{
           // q_home (6‐vector):
           (VectorXd(6) << -0.4488354, -2.02711196, 1.64630026, -1.18999615,
            -1.57079762, -2.01963249)
               .finished(),
           // dof:
           6,
           // joint_limits (2×6):
           (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 6) <<
                // low 6‐tuple
                -2 * M_PI,
            -2 * M_PI, -1 * M_PI, -2 * M_PI, -2 * M_PI, -2 * M_PI,
            // high 6‐tuple
            2 * M_PI, 2 * M_PI, 1 * M_PI, 2 * M_PI, 2 * M_PI, 2 * M_PI)
               .finished()}},
      // -------------- XArm7 --------------
      {XArm7, RobotMetaConfig{
        // q_home (7‐vector):
        (VectorXd(7) << 0, 0, 0, 0, 0, 0, 0).finished(),
        // dof:
        7,
        // joint_limits (2×7):
        (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 7) <<
            // low 7‐tuple
            -2 * M_PI, -2.094395, -2 * M_PI, -3.92699, -2 * M_PI, -M_PI, -2 * M_PI,
            // high 7‐tuple
             2 * M_PI,  2.059488,  2 * M_PI,  0.191986, 2 * M_PI, 1.692969, 2 * M_PI).finished()}},
      // -------------- SO101 --------------
      {SO101,
       RobotMetaConfig{
           // q_home (5‐vector):
           (VectorXd(5) << -9.40612320177057, -99.66130397967824,
            99.9124726477024, 69.96996996996998, -9.095744680851055)
               .finished(),
           // dof:
           5,
           // joint_limits (2×5):
           (Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>(2, 5) <<
                // low 5‐tuple
                -100.0,
            -100.0, -100.0, -100.0, -100.0,
            // high 5‐tuple
            100.0, 100.0, 100.0, 100.0, 100.0)
               .finished()}}}};

struct RobotConfig {
  RobotType robot_type = RobotType::FR3;
  RobotPlatform robot_platform = RobotPlatform::SIMULATION;
  virtual ~RobotConfig(){};
};
struct RobotState {
  virtual ~RobotState(){};
};

struct GripperConfig {
  virtual ~GripperConfig(){};
};
struct GripperState {
  virtual ~GripperState(){};
};

class Robot {
 public:
  virtual ~Robot(){};

  // Also add an implementation specific set_parameters function that takes
  // a deduced config type
  // This functino can also be used for signaling e.g. error recovery and the
  // like
  // bool set_parameters(const RConfig& cfg);

  virtual RobotConfig* get_parameters() = 0;

  virtual RobotState* get_state() = 0;

  virtual Pose get_cartesian_position() = 0;

  virtual void set_joint_position(const VectorXd& q) = 0;

  virtual VectorXd get_joint_position() = 0;

  virtual void move_home() = 0;

  virtual void reset() = 0;

  virtual void set_cartesian_position(const Pose& pose) = 0;

  virtual std::optional<std::shared_ptr<IK>> get_ik() = 0;

  common::Pose to_pose_in_world_coordinates(
      const Pose& pose_in_robot_coordinates);

  common::Pose to_pose_in_robot_coordinates(
      const Pose& pose_in_world_coordinates);

  virtual common::Pose get_base_pose_in_world_coordinates() = 0;
};

class Gripper {
 public:
  virtual ~Gripper(){};

  // Also add an implementation specific set_parameters function that takes
  // a deduced config type
  // bool set_parameters(const GConfig& cfg);

  virtual GripperConfig* get_parameters() = 0;
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
};

}  // namespace common
}  // namespace rcs
#endif  // RCS_ROBOT_H
