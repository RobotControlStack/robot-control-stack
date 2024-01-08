#include <franka/robot.h>
#include <franka/gripper.h>
#include <string>
#include <cmath>
#include <Eigen/Core>

using Vec7 = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
using Vec3 = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
using Vec4 = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;

using Mat3x3 = Eigen::Matrix<double, 3, 3, Eigen::ColMajor>;
using Mat4x4 = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>;

class FR3
{
private:
    /* data */
    bool guiding_mode;
    Vec7 q_home;

public:
    franka::Robot robot;
    franka::Gripper gripper;

    // TODO(tobi): use gripper, do we handle with null?
    FR3(const std::string ip);
    ~FR3();

    Vec7 get_joint_state();

    Mat4x4 get_ee_state();

    std::tuple<Vec3, Mat3x3> get_ee_pose();

    // use option to start new thread
    void set_joints(const Vec7 q_goal, const double speed_factor = 0.1);

    void set_ee_pose();

    // home and zero poses
    void move_home(double speed_factor = 0.1);

    void move_store_pose(double speed_factor = 0.1);

    void move_extend_arm(double speed_factor = 0.1);

    // gripper, methods should only work if gripper is defined
    void gripper_state();
    void gripper_homing();
    void gripper_max_grasping_width();
    void gripper_grasp();
    void gripper_is_grasped();
    void gripper_release();

    // TODO(tobi): abstract away end effector tool:
    // gripper and pump are possible

    // TODO(tobi): is there a way to access the buttons?

    // set robot to free movable
    void set_guiding_mode(bool enabled);
    bool get_guiding_mode();

    void automatic_error_recovery();

    // TODO(tobi): forward kinematic with model?

    // TODO(tobi): inverse kinematics and bounding box checking
    // of all cartesian positions of the robot in 3D space
};
