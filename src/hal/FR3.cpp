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
#include <thread>

const size_t DOF = 7;
const double SPEED_FACTOR = 0.2;

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

bool FR3::setParameters(double speed_factor)
{
    if (0.0 < speed_factor && speed_factor <= 1.0)
    {
        this->speed_factor = speed_factor;
        return true;
    } else {
        return false;
    }
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
    // other lib uses O_T_EE_c
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transMatrix(state.O_T_EE_c.data());
    rl::math::Transform x;
    x.matrix() = transMatrix;
    return x;
}
Eigen::Matrix<double, 4, 4, Eigen::ColMajor> FR3::getCartesianPosition2()
{
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> re(getCartesianPosition().matrix());
    return re;
}

// Methods from JointPositionActuator
void FR3::setJointPosition(const rl::math::Vector &q)
{
    // TODO: how does this motion generator work
    MotionGenerator motion_generator(speed_factor, q);
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

void FR3::automatic_error_recovery()
{
    robot.automaticErrorRecovery();
}

void FR3::wait_milliseconds(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void FR3::double_tap_robot_to_continue() {
    auto s = robot.readOnce();
    int touch_counter = false;
    bool can_be_touched_again = true;
    Eigen::Vector3d start_force;
    auto last_time_something_happened = std::chrono::system_clock::now();
    auto last_time_something_touched = std::chrono::system_clock::now();
    start_force << s.O_F_ext_hat_K[0], s.O_F_ext_hat_K[1], s.O_F_ext_hat_K[2];
    robot.read(
            [&](
                    const franka::RobotState &robot_state) {
                Eigen::Vector3d force;
                force << robot_state.O_F_ext_hat_K[0], robot_state.O_F_ext_hat_K[1], robot_state.O_F_ext_hat_K[2];
                if ((start_force - force).norm() > 2 and can_be_touched_again) {
                    touch_counter++;
                    can_be_touched_again = false;
                    last_time_something_happened = std::chrono::system_clock::now();
                    last_time_something_touched = std::chrono::system_clock::now();
                }
                if (touch_counter > 0 and (start_force - force).norm() < 1) {
                    can_be_touched_again = true;
                    last_time_something_happened = std::chrono::system_clock::now();
                }
                std::chrono::duration<double> elapse_time_touched =
                        std::chrono::system_clock::now() - last_time_something_touched;
                if (elapse_time_touched.count() > 0.5) {
                    touch_counter = 0;
                }
                std::chrono::duration<double> elapse_time_happend =
                        std::chrono::system_clock::now() - last_time_something_happened;
                if (elapse_time_happend.count() > 2) {
                    start_force = force;
                    last_time_something_happened = std::chrono::system_clock::now();

                }
                return touch_counter < 2;
            });
    wait_milliseconds(100);
}

rl::math::Transform interpolate(const rl::math::Transform &start_pose, const rl::math::Transform &dest_pose, double progress) {
    if (progress > 1) {
        progress = 1;
    }
    Eigen::Vector3d pos_result = start_pose.translation() + (dest_pose.translation() - start_pose.translation()) * progress;
    Eigen::Quaterniond quat_start, quat_end, quat_result;
    quat_start = start_pose.rotation();
    quat_end = dest_pose.rotation();
    quat_result = quat_start.slerp(progress, quat_end);
    rl::math::Transform result_pose;
    result_pose.translation() = pos_result;
    // result_pose.linear() = quat_result.toRotationMatrix();
    // result_pose.affine() = quat_result.toRotationMatrix();
    // result_pose.rotation() = quat_result.toRotationMatrix();
    result_pose.rotate(quat_result);

    // result_pose.makeAffine();
    return result_pose;
}

// todo this should be done with library function
std::array<double, 16> to_matrix(rl::math::Transform transform){
    Eigen::Matrix3d rot_mat = transform.rotation();
    std::array<double, 16> mat = {rot_mat(0, 0), rot_mat(1, 0), rot_mat(2, 0), 0, rot_mat(0, 1), rot_mat(1, 1),
                                  rot_mat(2, 1), 0, rot_mat(0, 2), rot_mat(1, 2), rot_mat(2, 2), 0, transform.translation().x(),
                                  transform.translation().y(), transform.translation().z(), 1};
    return mat;
}

void FR3::move_cartesian(Eigen::Matrix<double, 4, 4> dest, double max_time, std::optional<double> elbow, double max_force) {
    rl::math::Transform initial_pose = getCartesianPosition();


    rl::math::Transform destt(dest);

    // TODO: make max force check optional

    auto stop_condition = [&max_force](const franka::RobotState &state, const double progress) {
        Eigen::Vector3d force;
        force << state.O_F_ext_hat_K[0], state.O_F_ext_hat_K[1], state.O_F_ext_hat_K[2];
        double minimum_progress = 0.1;
        return force.norm() > max_force and progress > minimum_progress;
    };
    
    std::array<double, 2> initial_elbow;

    double time = 0.0;

    bool should_stop = false;

    robot.control(
            [=, &initial_elbow, &time, &max_time, &initial_pose, &should_stop](
                    const franka::RobotState &state,
                    franka::Duration time_step) -> franka::CartesianPose {
                time += time_step.toSec();
                if (time == 0) {
                    initial_elbow = state.elbow_c;
                }
                auto new_elbow = initial_elbow;
                const double progress = time / max_time;

                // calculate new pose
                rl::math::Transform new_pose_pose = interpolate(initial_pose, destt, progress);                    

                std::array<double, 16> new_pose = to_matrix(new_pose_pose);
                if (elbow.has_value()) {
                    new_elbow[0] += (elbow.value() - initial_elbow[0]) * (1 - std::cos(M_PI * progress)) / 2.0;
                }
                // if (stop_condition.is_initialized()) {
                //     should_stop = stop_condition.value()(generatorInput);
                // }
                should_stop = stop_condition(state, progress);
                if (time >= max_time or should_stop) {
                    if (elbow.has_value()) {
                        return franka::MotionFinished({new_pose, new_elbow});
                    }
                    return franka::MotionFinished(new_pose);
                }
                if (elbow.has_value()) {
                    return {new_pose, new_elbow};
                }
                return new_pose;
            }, franka::ControllerMode::kCartesianImpedance);
}