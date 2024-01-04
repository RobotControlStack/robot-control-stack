#include <franka/robot.h>
#include <Eigen/Core>
#include <string>
#include <memory>

#include "fr3.h"
#include "motion_generator.h"

// TODO(tobi): move this to some sort of utils file, include
// the same definition from motion_generator.h
using Vec7 = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
using Vec3 = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
using Vec4 = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;

using Mat3x3 = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
using Mat4x4 = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;


FR3::FR3(const std::string ip)
: robot(ip), gripper(ip)
{
    guiding_mode = false;
}

// TODO(tobi): Do we need a destructor?
FR3::~FR3()
{
}


std::shared_ptr<Vec7> FR3::get_joint_state(){
    franka::RobotState state = robot.readOnce();
    return std::make_shared<Vec7>(state.q_d.data());
}

std::shared_ptr<Mat4x4> FR3::get_ee_state(){
    franka::RobotState state = robot.readOnce();
    return std::make_shared<Mat4x4>(state.O_T_EE.data());
}
// TODO: return point in space


/**
 * TODO: use option to start new thread
 * @param[in] speed_factor: Must be between 0 and 1
 */
void FR3::set_joints(const Vec7 q_goal, double speed_factor){
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator);
}
void FR3::set_ee_pose(){}


void FR3::move_home(double speed_factor){
    set_joints(q_home, speed_factor);
}

void FR3::move_zero(double speed_factor){
    set_joints(q_zero, speed_factor);
}

// gripper, methods should only work if gripper is defined
void FR3::gripper_state(){}
void FR3::gripper_homing(){}
void FR3::gripper_max_grasping_width(){}
void FR3::gripper_grasp(){}
void FR3::gripper_is_grasped(){}
void FR3::gripper_release(){}

// set robot to free movable
void FR3::set_guiding_mode(bool enabled){
    std::array<bool, 6> activated;
    activated.fill(enabled);
    robot.setGuidingMode(activated, enabled);
    guiding_mode = enabled;
}

bool FR3::get_guiding_mode(){
    return guiding_mode;
}

void FR3::automatic_error_recovery(){
    // execute this when a franka::ControlException
    robot.automaticErrorRecovery();
}