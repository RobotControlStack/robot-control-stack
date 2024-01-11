#include <franka/robot.h>
#include <franka/gripper.h>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include "FR3.h"
#include "motion_generator.h"

const size_t DOF = 7;
const double SPEED_FACTOR = 0.5;

FR3::FR3(const std::string ip) :
    AxisController(DOF),
    CartesianPositionActuator(DOF),
    CartesianPositionSensor(DOF),
    Gripper(),
    JointPositionActuator(DOF),
    JointPositionSensor(DOF),
    robot(ip),
    gripper(ip)
{
    setDefaultBehavior(robot);
}

FR3::~FR3()
{
}

// Methods from Device
void close(){}
void open(){}
void start(){}
void stop(){}

// Methods from CartesianPositionActuator
void setCartesianPosition(const ::rl::math::Transform& x){
    // TODO: control loop for cartesian position
    // TODO: trajectory planner
}

// Methods from CartesianPositionSensor
rl::math::Transform getCartesianPosition(){
    franka::RobotState state = robot.readOnce();
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transMatrix(state.O_T_EE.data());
    rl::math::Transform x;
    x.matrix() = transMatrix;
    return x;
}

// Methods from Gripper
void halt(){}
void release(){}
void shut(){}

// Methods from JointPositionActuator
void setJointPosition(const rl::math::Vector& q){
    // TODO: how does this motion generator work
    MotionGenerator motion_generator(SPEED_FACTOR, q);
    robot.control(motion_generator);
}

// Methods from JointPositionSensor
rl::math::Vector getJointPosition(){
    franka::RobotState state = robot.readOnce();
    return rl::math::Vector(state.q.data());
}
