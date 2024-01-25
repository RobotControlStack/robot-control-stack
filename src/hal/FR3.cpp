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
#include <rl/mdl/Kinematic.h>
#include <memory>
#include <rl/mdl/Exception.h>
#include "FR3.h"
#include "motion_generator.h"

const size_t DOF = 7;
const double SPEED_FACTOR = 0.5;

FR3::FR3(const std::string &ip, const std::string &filename) : AxisController(DOF),
                                                             CartesianPositionActuator(DOF),
                                                             CartesianPositionSensor(DOF),
                                                             JointPositionActuator(DOF),
                                                             JointPositionSensor(DOF),
                                                             robot(ip),
                                                             model(),
                                                             q_home((Vec7() << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished())
{
    // set collision behavior and impedance
    setDefaultRobotBehavior();

    // todo: move this into initialization list
    rl::mdl::UrdfFactory factory;
    factory.load(filename, &model);
    ik = std::make_unique<rl::mdl::JacobianInverseKinematics>(static_cast<rl::mdl::Kinematic *>(&model));
}

FR3::~FR3()
{
}

void FR3::setDefaultRobotBehavior()
{
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

// Methods from Device
void FR3::close() {}
void FR3::open() {}
void FR3::start() {}
void FR3::stop() {}

// Methods from CartesianPositionActuator
void FR3::setCartesianPosition(const ::rl::math::Transform &x)
{
    // TODO: control loop for cartesian position
    // TODO: trajectory planner
    ik->addGoal(x, 0);
    bool success = ik->solve();
    if (success)
    {
        rl::math::Vector q(6);
        q = model.getPosition();
        setJointPosition(q);
    }
    else
    {
        // throw error
        throw rl::mdl::Exception("IK failed");
    }
}

// Methods from CartesianPositionSensor
rl::math::Transform FR3::getCartesianPosition() const
{
    // This const cast is necessary because the readOnce method is not const.
    // We currently do not know if the state of the robot object changes.
    franka::RobotState state = const_cast<franka::Robot*>(&robot)->readOnce();
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transMatrix(state.O_T_EE.data());
    rl::math::Transform x;
    x.matrix() = transMatrix;
    return x;
}

// Methods from JointPositionActuator
void FR3::setJointPosition(const rl::math::Vector &q)
{
    // TODO: how does this motion generator work
    MotionGenerator motion_generator(SPEED_FACTOR, q);
    robot.control(motion_generator);
}

// Methods from JointPositionSensor
rl::math::Vector FR3::getJointPosition() const
{
    // TODO: validate that the state of the robot object does not change
    // This const cast is necessary because the readOnce method is not const.
    // We currently do not know if the state of the robot object changes.
    franka::RobotState state = const_cast<franka::Robot*>(&robot)->readOnce();
    Vec7 joints(state.q.data());
    return joints;
}


// void FR3::setGuidingMode(bool enabled)
// {
//     std::array<bool, 6> activated;
//     activated.fill(enabled);
//     robot.setGuidingMode(activated, enabled);
// }
void FR3::setGuidingMode(std::array<bool, 6> activated, bool enabled){
    robot.setGuidingMode(activated, enabled);
}
void FR3::move_home()
{
    setJointPosition(q_home);
}
