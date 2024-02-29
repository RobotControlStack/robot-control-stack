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

namespace rcs {
namespace hw {

Vector7d q_home(
    (Vector7d() << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished());
double DEFAULT_SPEED_FACTOR = 0.2;

typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;
typedef Eigen::Matrix<int, 7, 1, Eigen::ColMajor> Vector7i;
struct FR3Load {
  double load_mass;
  std::optional<Eigen::Vector3d> f_x_cload;
  std::optional<Eigen::Matrix3f> load_inertia;
};
enum IKController { internal, robotics_library };

class FR3 {
 private:
  franka::Robot robot;
  rl::mdl::Dynamic model;
  std::optional<std::unique_ptr<rl::mdl::JacobianInverseKinematics>> ik;
  double speed_factor = DEFAULT_SPEED_FACTOR;
  bool guiding_mode_enabled;
  // TODO: add max force

 public:
  FR3(const std::string &ip,
      std::optional<const std::string &> filename = std::nullopt);
  ~FR3();

  bool set_parameters(std::optional<double> speed_factor,
                      std::optional<const FR3Load &> load_parameters);

  void set_default_robot_behavior();

  Eigen::Affine3d get_cartesian_position();
  //   Eigen::Matrix<double, 4, 4, Eigen::ColMajor> getCartesianPosition2();

  void set_joint_position(const Vector7d &q);

  Vector7d get_joint_position();

  void set_guiding_mode(bool enabled);

  void move_home();

  void automatic_error_recovery();

  static void wait_milliseconds(int milliseconds);

  void double_tap_robot_to_continue();

  void set_cartesian_position(
      const ::Eigen::Affine3d &x, IKController controller,
      const std::optional<Eigen::Affine3d &> nominal_end_effector_frame);

  void set_cartesian_position_internal(const Eigen::Affine3d &dest,
                                       double max_time,
                                       std::optional<double> elbow,
                                       std::optional<double> max_force = 5);

  void set_cartesian_position_rl(const ::Eigen::Affine3d &x);

  Eigen::Affine3d FR3::get_cartesian_position();
};
}  // namespace hw
}  // namespace rcs