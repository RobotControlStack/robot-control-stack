#include <franka/robot.h>
#include <string>
#include <cmath>
#include <Eigen/Core>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/UrdfFactory.h>
#include <memory>

class FR3 : public rl::hal::CartesianPositionActuator,
            public rl::hal::CartesianPositionSensor,
            public rl::hal::JointPositionActuator,
            public rl::hal::JointPositionSensor
{
private:
    franka::Robot robot;
    rl::mdl::Dynamic model;
    std::unique_ptr<rl::mdl::JacobianInverseKinematics> ik;

public:
    FR3(const std::string ip, const std::string filename);
    ~FR3();
    // how to do destructor of inherited classes? how to handle virtual descrutors

    void setDefaultRobotBehavior();

    // Methods from Device
    void close();
    void open();
    void start();
    void stop();

    // Methods from CartesianPositionActuator
    void setCartesianPosition(const ::rl::math::Transform &x);

    // Methods from CartesianPositionSensor
    rl::math::Transform getCartesianPosition() const;

    // Methods from JointPositionActuator
    void setJointPosition(const ::rl::math::Vector &q);

    // Methods from JointPositionSensor
    rl::math::Vector getJointPosition() const;
};
