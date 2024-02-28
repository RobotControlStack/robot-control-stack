#include <franka/gripper.h>
#include <rl/hal/CartesianPositionActuator.h>
#include <rl/hal/CartesianPositionSensor.h>
#include <rl/hal/Gripper.h>
#include <rl/hal/JointPositionActuator.h>
#include <rl/hal/JointPositionSensor.h>
#include <rl/math/Transform.h>

#include <Eigen/Core>
#include <cmath>
#include <string>

namespace rcs {
namespace hw {

class FrankaHand : public rl::hal::Gripper {
 private:
  franka::Gripper gripper;
  double grasping_width = 0;
  double speed = 0.1;
  double force = 10;

 public:
  FrankaHand(const std::string ip);
  ~FrankaHand();

  // Methods from Device
  void close();
  void open();
  void start();
  void stop();
  bool setParameters(double grapsing_width, double speed = 0.1,
                     double force = 10);
  std::tuple<double, double, bool> getState();

  // method that puts the gripper to certain position

  // Methods from Gripper
  void halt();
  void release();
  void shut();
};
}  // namespace hw
}  // namespace rcs