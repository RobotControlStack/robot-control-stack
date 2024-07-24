#include "Robot.h"
namespace rcs {
namespace common {
  common::Pose Robot::to_robot_frame(const Pose& pose) {
    return this->get_origin().inverse() * pose;
  }

  common::Pose Robot::to_world_frame(const Pose& pose) {
    return pose.inverse() * this->get_origin();
  } 
}
}
