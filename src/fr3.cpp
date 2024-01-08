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

/**
 * TODO: Implement store pose, cartesian control, change to std::array, pose where gripper looks up with cartesian control
 */

FR3::FR3(const std::string ip)
    : robot(ip), gripper(ip),
      q_home((Vec7() << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished()),
      guiding_mode(false)
{
    setDefaultBehavior(robot);
}

// TODO(tobi): Do we need a destructor?
FR3::~FR3()
{
}

Vec7 FR3::get_joint_state()
{
    franka::RobotState state = robot.readOnce();
    return Vec7(state.q.data());
}

Mat4x4 FR3::get_ee_state()
{
    franka::RobotState state = robot.readOnce();
    return Mat4x4(state.O_T_EE.data());
}
// TODO: function that returns end effector coordinates and orientation in world frame
std::tuple<Vec3, Mat3x3> FR3::get_ee_pose()
{
    Mat4x4 state = get_ee_state();
    Vec3 posiition = state.block<3, 1>(0, 3);
    Mat3x3 orientation = state.topLeftCorner<3, 3>();
    return std::make_tuple(posiition, orientation);
}

/**
 * @param[in] speed_factor: Must be between 0 and 1
 */
void FR3::set_joints(const Vec7 q_goal, double speed_factor)
{
    MotionGenerator motion_generator(speed_factor, q_goal);
    robot.control(motion_generator);
}
void FR3::set_ee_pose() {}

void FR3::move_home(double speed_factor)
{
    set_joints(q_home, speed_factor);
}

void FR3::move_extend_arm(double speed_factor)
{
    // zero position of the last joint is offset by 45 degrees
    Vec7 q_extend = (Vec7() << 0, 0, 0, -0.4461, 0, M_PI, M_PI_4).finished();
    set_joints(q_extend, speed_factor);
}

void FR3::move_store_pose(double speed_factor)
{
    // store postion from the manual p.95 (https://download.franka.de/documents/100010_Product%20Manual%20Franka%20Emika%20Robot_10.21_EN.pdf):
    // [0°, -32,08°, 0°, -170,17°, 0°, 0°, 45°]
    // in rads: [0., -0.55990162, 0., -2.97002679,  0., 0., 0.78539816]
    // only go into store postion AFTER the hand is dismounted

    // Store pose in which the robot came
    // [-0.3°, -20.9°, -0.3°, -175.5°, 90.4°, 89.8°, -17.9°]
    // in rads: [-0.00523599, -0.36477381, -0.00523599, -3.06305284, 1.57777764, 1.56730567, -0.31241394]
    // 4th joint is outside of the recommanded joint range which means that the joint velocity needs to be
    // adapted according to https://frankaemika.github.io/docs/control_parameters.html#control-parameters-specifications

    Vec7 q_store = (Vec7() << -0.00523599, -0.36477381, -0.00523599, -3.06305284, 1.57777764, 1.56730567, -0.31241394).finished();
    set_joints(q_store, speed_factor);
}

// gripper, methods should only work if gripper is defined
void FR3::gripper_state() {}
void FR3::gripper_homing() {}
void FR3::gripper_max_grasping_width() {}
void FR3::gripper_grasp() {}
void FR3::gripper_is_grasped() {}
void FR3::gripper_release() {}

// set robot to free movable
void FR3::set_guiding_mode(bool enabled)
{
    std::array<bool, 6> activated;
    activated.fill(enabled);
    robot.setGuidingMode(activated, enabled);
    guiding_mode = enabled;
}

bool FR3::get_guiding_mode()
{
    return guiding_mode;
}

void FR3::automatic_error_recovery()
{
    // execute this when a franka::ControlException
    robot.automaticErrorRecovery();
}