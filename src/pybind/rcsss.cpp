#include <common/NRobotsWithGripper.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <hw/FR3.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = R"pbdoc(
        Robot Control Stack Python Bindings
        -----------------------

        .. currentmodule:: rcsss

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  // COMMON MODULE
  auto common = m.def_submodule("common", "common module");
  py::class_<rcs::common::Pose>(common, "Pose")
      .def(py::init<>())
      .def(py::init<const Eigen::Matrix4d &>(), py::arg("pose"))
      .def(py::init<const Eigen::Matrix3d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY &, const Eigen::Vector3d &>(),
           py::arg("quaternion"), py::arg("translation"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rotation_rpy", &rcs::common::Pose::rotation_rpy)
      .def("interpolate", &rcs::common::Pose::interpolate, py::arg("dest_pose"),
           py::arg("progress"))
      .def("__str__", &rcs::common::Pose::str)
      .def(py::self * py::self);

  py::class_<rcs::common::RPY>(common, "RPY")
      .def(py::init<double, double, double>(), py::arg("roll") = 0.0,
           py::arg("pitch") = 0.0, py::arg("yaw") = 0.0)
      .def_readwrite("roll", &rcs::common::RPY::roll)
      .def_readwrite("pitch", &rcs::common::RPY::pitch)
      .def_readwrite("yaw", &rcs::common::RPY::yaw)
      .def("rotation_matrix", &rcs::common::RPY::rotation_matrix)
      .def("__str__", &rcs::common::RPY::str)
      .def(py::self + py::self);

  py::class_<rcs::common::RConfig, std::shared_ptr<rcs::common::RConfig>>(
      common, "RConfig");
  py::class_<rcs::common::RState, std::shared_ptr<rcs::common::RState>>(
      common, "RState");
  py::class_<rcs::common::GConfig, std::shared_ptr<rcs::common::GConfig>>(
      common, "GConfig");
  py::class_<rcs::common::GState, std::shared_ptr<rcs::common::GState>>(
      common, "GState");

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, rcs::common::PyRobot<>,
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

  py::class_<rcs::common::Gripper, rcs::common::PyGripper<>,
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

  py::class_<rcs::hw::FR3State>(hw, "FR3State");
  py::class_<rcs::hw::FR3Load>(hw, "FR3Load")
      .def_readwrite("load_mass", &rcs::hw::FR3Load::load_mass)
      .def_readwrite("f_x_cload", &rcs::hw::FR3Load::f_x_cload)
      .def_readwrite("load_inertia", &rcs::hw::FR3Load::load_inertia);

  py::enum_<rcs::hw::IKController>(hw, "IKController")
      .value("internal", rcs::hw::IKController::internal)
      .value("robotics_library", rcs::hw::IKController::robotics_library)
      .export_values();

  py::class_<rcs::hw::FR3Config, std::shared_ptr<rcs::hw::FR3Config>>(
      hw, "FR3Config")
      .def_readwrite("controller", &rcs::hw::FR3Config::controller)
      .def_readwrite("guiding_mode_enabled",
                     &rcs::hw::FR3Config::guiding_mode_enabled)
      .def_readwrite("load_parameters", &rcs::hw::FR3Config::load_parameters)
      .def_readwrite("nominal_end_effector_frame",
                     &rcs::hw::FR3Config::nominal_end_effector_frame);

  py::class_<rcs::hw::FR3, rcs::common::Robot,
             rcs::common::PyRobot<rcs::hw::FR3>, std::shared_ptr<rcs::hw::FR3>>(
      hw, "FR3")
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
#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
