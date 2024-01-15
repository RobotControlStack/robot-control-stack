#include <franka/vacuum_gripper.h>
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


FrankaHand::FrankaHand(const std::string ip) :
    Gripper(),
    gripper(ip)
{
}

FrankaHand::~FrankaHand()
{
}

// Methods from Device
void FrankaHand::close(){}
void FrankaHand::open(){}
void FrankaHand::start(){}
void FrankaHand::stop(){}

// Methods from Gripper
void FrankaHand::halt(){}
void FrankaHand::release(){}
void FrankaHand::shut(){}