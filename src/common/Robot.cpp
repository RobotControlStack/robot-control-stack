#include "Robot.h"
namespace rcs {
namespace common {

common::Pose Robot::to_pose_in_robot_coordinates(const Pose& pose_in_world_coordinates) {
  return this->get_base_pose_in_world_coordinates().inverse() * pose_in_world_coordinates;
}

common::Pose Robot::to_pose_in_world_coordinates(const Pose& pose_in_robot_coordinates) {
  return this->get_base_pose_in_world_coordinates() * pose_in_robot_coordinates;
}
}  // namespace common
}  // namespace rcs
