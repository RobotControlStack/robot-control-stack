#include <franka/vacuum_gripper.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>

#include <Eigen/Core>
#include <cmath>
#include <string>

class FrankaHand : public rl::hal::Gripper {
 private:
  franka::VacuumGripper gripper;

 public:
  FrankaHand(const std::string ip);
  ~FrankaHand();

  // Methods from Device
  void close();
  void open();
  void start();
  void stop();

  // Methods from Gripper
  void halt();
  void release();
  void shut();
};