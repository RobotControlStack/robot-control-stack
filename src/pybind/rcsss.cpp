#include <common/NRobotsWithGripper.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <hw/FR3.h>
#include <hw/FrankaHand.h>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sim/FR3.h>

#include <memory>

#include "rl/mdl/UrdfFactory.h"

//#include "mujoco/raw.h"
//#include "mujoco/structs.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

/**
 * @brief Robot trampoline class for python bindings,
 * needed for pybind11 to override virtual functions,
 * see
 * https://pybind11.readthedocs.io/en/stable/advanced/classes.html#virtual-and-inheritance
 */
template <class RobotBase = rcs::common::Robot>
class PyRobot : public RobotBase {
 public:
  using RobotBase::RobotBase;  // Inherit constructors

  bool set_parameters(const rcs::common::RConfig &cfg) override {
    PYBIND11_OVERRIDE_PURE(bool, RobotBase, set_parameters, cfg);
  }

  std::unique_ptr<rcs::common::RConfig> get_parameters() override {
    PYBIND11_OVERRIDE_PURE(std::unique_ptr<rcs::common::RConfig>, RobotBase,
                           get_parameters, );
  }

  std::unique_ptr<rcs::common::RState> get_state() override {
    PYBIND11_OVERRIDE_PURE(std::unique_ptr<rcs::common::RState>, RobotBase,
                           get_state, );
  }

  rcs::common::Pose get_cartesian_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, RobotBase,
                           get_cartesian_position, );
  }

  void set_joint_position(const rcs::common::Vector7d &q) override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, set_joint_position, q);
  }

  rcs::common::Vector7d get_joint_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Vector7d, RobotBase,
                           get_joint_position, );
  }

  void move_home() override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, move_home, );
  }

  void set_cartesian_position(const rcs::common::Pose &pose) override {
    PYBIND11_OVERRIDE_PURE(void, RobotBase, set_cartesian_position, pose);
  }
};

/**
 * @brief Gripper trampoline class for python bindings
 */
template <class GripperBase = rcs::common::Gripper>
class PyGripper : public GripperBase {
 public:
  using GripperBase::GripperBase;  // Inherit constructors

  bool set_parameters(const rcs::common::GConfig &cfg) override {
    PYBIND11_OVERRIDE_PURE(bool, GripperBase, set_parameters, cfg);
  }

  std::unique_ptr<rcs::common::GConfig> get_parameters() override {
    PYBIND11_OVERRIDE_PURE(std::unique_ptr<rcs::common::GConfig>, GripperBase,
                           get_parameters, );
  }

  std::unique_ptr<rcs::common::GState> get_state() override {
    PYBIND11_OVERRIDE_PURE(std::unique_ptr<rcs::common::GState>, GripperBase,
                           get_state, );
  }

  bool grasp() override { PYBIND11_OVERRIDE_PURE(bool, GripperBase, grasp, ); }

  void release() override {
    PYBIND11_OVERRIDE_PURE(void, GripperBase, release, );
  }

  void shut() override { PYBIND11_OVERRIDE_PURE(void, GripperBase, shut, ); }
};

PYBIND11_MODULE(_core, m) {
  m.doc() = R"pbdoc(
        Robot Control Stack Python Bindings
        -----------------------

        .. currentmodule:: _core

        .. autosummary::
           :toctree: _generate

    )pbdoc";
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif

  // COMMON MODULE
  auto common = m.def_submodule("common", "common module");
  py::class_<rcs::common::RPY>(common, "RPY")
      .def(py::init<double, double, double>(), py::arg("roll") = 0.0,
           py::arg("pitch") = 0.0, py::arg("yaw") = 0.0)
      .def_readwrite("roll", &rcs::common::RPY::roll)
      .def_readwrite("pitch", &rcs::common::RPY::pitch)
      .def_readwrite("yaw", &rcs::common::RPY::yaw)
      .def("rotation_matrix", &rcs::common::RPY::rotation_matrix)
      .def("__str__", &rcs::common::RPY::str)
      .def(py::self + py::self);

  py::class_<rcs::common::Pose>(common, "Pose")
      .def(py::init<>())
      .def(py::init<const Eigen::Matrix4d &>(), py::arg("pose"))
      .def(py::init<const Eigen::Matrix3d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &, const Eigen::Vector3d &>(),
           py::arg("quaternion"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY &, const Eigen::Vector3d &>(),
           py::arg("rpy"), py::arg("translation"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rotation_rpy", &rcs::common::Pose::rotation_rpy)
      .def("interpolate", &rcs::common::Pose::interpolate, py::arg("dest_pose"),
           py::arg("progress"))
      .def("__str__", &rcs::common::Pose::str)
      .def(py::self * py::self);

  py::class_<rcs::common::RConfig>(common, "RConfig");
  py::class_<rcs::common::RState>(common, "RState");
  py::class_<rcs::common::GConfig>(common, "GConfig");
  py::class_<rcs::common::GState>(common, "GState");

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, PyRobot<>,
             std::shared_ptr<rcs::common::Robot>>(common, "Robot")
      .def("set_parameters", &rcs::common::Robot::set_parameters,
           py::arg("cfg"))
      .def("get_parameters", &rcs::common::Robot::get_parameters)
      .def("get_state", &rcs::common::Robot::get_state)
      .def("get_cartesian_position",
           &rcs::common::Robot::get_cartesian_position)
      .def("set_joint_position", &rcs::common::Robot::set_joint_position,
           py::arg("q"))
      .def("get_joint_position", &rcs::common::Robot::get_joint_position)
      .def("move_home", &rcs::common::Robot::move_home)
      .def("set_cartesian_position",
           &rcs::common::Robot::set_cartesian_position, py::arg("pose"));

  py::class_<rcs::common::Gripper, PyGripper<>,
             std::shared_ptr<rcs::common::Gripper>>(common, "Gripper")
      .def("set_parameters", &rcs::common::Gripper::set_parameters,
           py::arg("cfg"))
      .def("get_parameters", &rcs::common::Gripper::get_parameters)
      .def("get_state", &rcs::common::Gripper::get_state)
      .def("grasp", &rcs::common::Gripper::grasp)
      .def("release", &rcs::common::Gripper::release)
      .def("shut", &rcs::common::Gripper::shut);

  py::class_<rcs::common::RobotWithGripper,
             std::shared_ptr<rcs::common::RobotWithGripper>>(common,
                                                             "RobotWithGripper")
      .def(py::init<std::shared_ptr<rcs::common::Robot>,
                    std::optional<std::shared_ptr<rcs::common::Gripper>>>(),
           py::arg("robot"), py::arg("gripper"));

  py::class_<rcs::common::NRobotsWithGripper>(common, "NRobotsWithGripper")
      .def(py::init<
               std::vector<std::shared_ptr<rcs::common::RobotWithGripper>>>(),
           py::arg("robots_with_gripper"))
      .def("set_parameters_r",
           &rcs::common::NRobotsWithGripper::set_parameters_r, py::arg("idxs"),
           py::arg("cfgs"))
      .def("get_parameters_r",
           &rcs::common::NRobotsWithGripper::get_parameters_r, py::arg("idxs"))
      .def("get_state_r", &rcs::common::NRobotsWithGripper::get_state_r,
           py::arg("idxs"))
      .def("get_cartesian_position",
           &rcs::common::NRobotsWithGripper::get_cartesian_position,
           py::arg("idxs"))
      .def("set_joint_position",
           &rcs::common::NRobotsWithGripper::set_joint_position,
           py::arg("idxs"), py::arg("q"))
      .def("get_joint_position",
           &rcs::common::NRobotsWithGripper::get_joint_position,
           py::arg("idxs"))
      .def("move_home", &rcs::common::NRobotsWithGripper::move_home,
           py::arg("idxs"))
      .def("set_cartesian_position",
           &rcs::common::NRobotsWithGripper::set_cartesian_position,
           py::arg("idxs"), py::arg("pose"))
      .def("set_parameters_g",
           &rcs::common::NRobotsWithGripper::set_parameters_g, py::arg("idxs"),
           py::arg("cfgs"))
      .def("get_parameters_g",
           &rcs::common::NRobotsWithGripper::get_parameters_g, py::arg("idxs"))
      .def("get_state_g", &rcs::common::NRobotsWithGripper::get_state_g,
           py::arg("idxs"))
      .def("grasp", &rcs::common::NRobotsWithGripper::grasp, py::arg("idxs"))
      .def("release", &rcs::common::NRobotsWithGripper::release,
           py::arg("idxs"))
      .def("shut", &rcs::common::NRobotsWithGripper::shut, py::arg("idxs"));

  // HARDWARE MODULE
  auto hw = m.def_submodule("hw", "hardware module");

  py::class_<rcs::hw::FR3State, rcs::common::RState>(hw, "FR3State")
      .def(py::init<>());
  py::class_<rcs::hw::FR3Load>(hw, "FR3Load")
      .def(py::init<>())
      .def_readwrite("load_mass", &rcs::hw::FR3Load::load_mass)
      .def_readwrite("f_x_cload", &rcs::hw::FR3Load::f_x_cload)
      .def_readwrite("load_inertia", &rcs::hw::FR3Load::load_inertia);

  py::enum_<rcs::hw::IKController>(hw, "IKController")
      .value("internal", rcs::hw::IKController::internal)
      .value("robotics_library", rcs::hw::IKController::robotics_library)
      .export_values();

  py::class_<rcs::hw::FR3Config, rcs::common::RConfig>(hw, "FR3Config")
      .def(py::init<>())
      .def_readwrite("controller", &rcs::hw::FR3Config::controller)
      .def_readwrite("speed_factor", &rcs::hw::FR3Config::speed_factor)
      .def_readwrite("guiding_mode_enabled",
                     &rcs::hw::FR3Config::guiding_mode_enabled)
      .def_readwrite("load_parameters", &rcs::hw::FR3Config::load_parameters)
      .def_readwrite("nominal_end_effector_frame",
                     &rcs::hw::FR3Config::nominal_end_effector_frame);

  py::class_<rcs::hw::FHConfig, rcs::common::GConfig>(hw, "FHConfig")
      .def(py::init<>())
      .def_readwrite("grasping_width", &rcs::hw::FHConfig::grasping_width)
      .def_readwrite("speed", &rcs::hw::FHConfig::speed)
      .def_readwrite("force", &rcs::hw::FHConfig::force)
      .def_readwrite("epsilon_inner", &rcs::hw::FHConfig::epsilon_inner)
      .def_readwrite("epsilon_outer", &rcs::hw::FHConfig::epsilon_outer);

  py::class_<rcs::hw::FHState, rcs::common::GState>(hw, "FHState")
      .def(py::init<>())
      .def_readonly("width", &rcs::hw::FHState::width)
      .def_readonly("is_grasped", &rcs::hw::FHState::is_grasped)
      .def_readonly("temperature", &rcs::hw::FHState::temperature);

  py::class_<rcs::hw::FR3, rcs::common::Robot, PyRobot<rcs::hw::FR3>,
             std::shared_ptr<rcs::hw::FR3>>(hw, "FR3")
      // No idea why the line below does not compile
      //  .def(py::init<const std::string &, const std::optional<std::string>
      //  &>(),
      //       py::arg("ip"), py::arg("filename") = std::nullopt)
      .def(py::init([](const std::string &ip,
                       const std::optional<std::string> &filename) {
             return std::shared_ptr<rcs::hw::FR3>(
                 new rcs::hw::FR3(ip, filename));
           }),
           py::arg("ip"), py::arg("filename") = std::nullopt)
      .def("set_default_robot_behavior",
           &rcs::hw::FR3::set_default_robot_behavior)
      .def("set_guiding_mode", &rcs::hw::FR3::set_guiding_mode,
           py::arg("enabled"))
      .def("automatic_error_recovery", &rcs::hw::FR3::automatic_error_recovery)
      .def("double_tap_robot_to_continue",
           &rcs::hw::FR3::double_tap_robot_to_continue)
      .def("set_cartesian_position_internal",
           &rcs::hw::FR3::set_cartesian_position_rl, py::arg("pose"))
      .def("set_cartesian_position_rl",
           &rcs::hw::FR3::set_cartesian_position_internal, py::arg("pose"),
           py::arg("max_time"), py::arg("elbow"), py::arg("max_force") = 5);

  py::class_<rcs::hw::FrankaHand, rcs::common::Gripper,
             PyGripper<rcs::hw::FrankaHand>,
             std::shared_ptr<rcs::hw::FrankaHand>>(hw, "FrankaHand")
      // No idea why the line below does not compile
      //   .def(py::init<const std::string&>(), py::arg("ip"))
      .def(py::init([](const std::string &ip) {
        return std::shared_ptr<rcs::hw::FrankaHand>(
            new rcs::hw::FrankaHand(ip));
      }))
      .def("homing", &rcs::hw::FrankaHand::homing);

  // SIM MODULE
  auto sim = m.def_submodule("sim", "sim module");
  py::class_<rcs::sim::FR3Config, rcs::common::RConfig>(sim, "FR3Config")
      .def(py::init<>())
      .def_readwrite("ik_duration", &rcs::sim::FR3Config::ik_duration)
      .def_readwrite("realtime", &rcs::sim::FR3Config::realtime)
      .def_readwrite("trajectory_trace",
                     &rcs::sim::FR3Config::trajectory_trace);

  py::class_<rcs::sim::FR3State, rcs::common::RState>(sim, "FR3State")
      .def(py::init<>())
      .def_readonly("collision", &rcs::sim::FR3State::collision)
      .def_readonly("ik_success", &rcs::sim::FR3State::ik_success);

  py::class_<rcs::sim::FR3, rcs::common::Robot, PyRobot<rcs::sim::FR3>,
             std::shared_ptr<rcs::sim::FR3>>(sim, "_FR3")
      .def(py::init([](long mjmdl, long mjdata, const std::string rlmdl,
                       std::optional<bool> render) {
             return std::make_shared<rcs::sim::FR3>(
                 (mjModel *)mjmdl, (mjData *)mjdata,
                 rl::mdl::UrdfFactory().create(rlmdl), render);
           }),
           py::arg("mjmdl"), py::arg("mjdata"), py::arg("rlmdl"),
           py::arg("render") = true)
      .def("reset", &rcs::sim::FR3::reset)
      .def("clear_markers", &rcs::sim::FR3::clear_markers);
}
