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


class FR3 : public rl::hal::CartesianPositionActuator,
            public rl::hal::CartesianPositionSensor,
            public rl::hal::Gripper,
            public rl::hal::JointPositionActuator,
            public rl::hal::JointPositionSensor
{
private:
    franka::Robot robot;
    franka::Gripper gripper;

public:
    FR3(const std::string ip);
    ~FR3();

    // Methods from Device
    void close();
    void open();
    void start();
    void stop();
    
    // Methods from CartesianPositionActuator
    void setCartesianPosition(const ::rl::math::Transform& x);

    // Methods from CartesianPositionSensor
    rl::math::Transform getCartesianPosition();

    // Methods from Gripper
    void halt();
    void release();
    void shut();

    // Methods from JointPositionActuator
    void setJointPosition(const ::rl::math::Vector& q);

    // Methods from JointPositionSensor
    rl::math::Vector getJointPosition();
};
