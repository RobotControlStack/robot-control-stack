#include <franka/robot.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/UrdfFactory.h>

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vec7;

class FR3 : public rl::hal::CartesianPositionActuator,
            public rl::hal::CartesianPositionSensor,
            public rl::hal::JointPositionActuator,
            public rl::hal::JointPositionSensor {
 private:
  franka::Robot robot;
  rl::mdl::Dynamic model;
  std::unique_ptr<rl::mdl::JacobianInverseKinematics> ik;
  Vec7 q_home;
  double speed_factor = 0.2;
  bool guiding_mode_enabled;

 public:
  FR3(const std::string &ip, const std::string &filename);
  ~FR3();
  // how to do destructor of inherited classes? how to handle virtual descrutors

  bool setParameters(double speed_factor);

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
  Eigen::Matrix<double, 4, 4, Eigen::ColMajor> getCartesianPosition2();

  // Methods from JointPositionActuator
  void setJointPosition(const ::rl::math::Vector &q);

  // Methods from JointPositionSensor
  rl::math::Vector getJointPosition() const;

  void setGuidingMode(bool enabled);

  // inertia?
  // void setLoad(double load_mass);
  // Own methods
  void move_home();

  void automatic_error_recovery();

  static void wait_milliseconds(int milliseconds);

  void double_tap_robot_to_continue();

  void move_cartesian(rl::math::Transform dest, double max_time,
                      std::optional<double> elbow,
                      std::optional<double> max_force = 5);
};
