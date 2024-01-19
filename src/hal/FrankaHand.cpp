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
#include "FrankaHand.h"
#include <tuple>

FrankaHand::FrankaHand(const std::string ip) : Gripper(),
                                               gripper(ip)
{
}

FrankaHand::~FrankaHand()
{
}

// Methods from Device
void FrankaHand::close() {}
void FrankaHand::open() {}
void FrankaHand::start()
{
    // Do a homing in order to estimate the maximum
    // grasping width with the current fingers.
    gripper.homing();
}
void FrankaHand::stop()
{
    gripper.stop();
}

bool FrankaHand::setParameters(double grapsing_width,
                               double speed = 0.1,
                               double force = 10)
{
    franka::GripperState gripper_state = gripper.readOnce();
    if (gripper_state.max_width < grapsing_width)
    {
        return false;
    }
    this->grasping_width = grasping_width;
    this->speed = speed;
    this->force = force;
    return true;
}

// Methods to read current state
std::tuple<double, double, bool> FrankaHand::getState()
{
    franka::GripperState gripper_state = gripper.readOnce();
    return std::make_tuple(gripper_state.width, gripper_state.max_width, gripper_state.is_grasped);
}

// Methods from Gripper
void FrankaHand::halt()
{
    gripper.grasp(grasping_width, speed, force);
}
void FrankaHand::release()
{
    franka::GripperState gripper_state = gripper.readOnce();
    gripper.move(gripper_state.max_width, speed);
}
void FrankaHand::shut()
{
    gripper.move(grasping_width, speed);
}
