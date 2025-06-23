#include <franka/exception.h>
#include <hw/FR3.h>
#include <hw/FrankaHand.h>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "rcs/IK.h"
#include "rcs/Pose.h"
#include "rcs/Robot.h"
#include "rcs/utils.h"
#include "rl/mdl/UrdfFactory.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

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

  // HARDWARE MODULE
  auto hw = m.def_submodule("hw", "rcs fr3 module");

  py::class_<rcs::hw::FR3State, rcs::common::RobotState>(hw, "FR3State")
      .def(py::init<>());
  py::class_<rcs::hw::FR3Load>(hw, "FR3Load")
      .def(py::init<>())
      .def_readwrite("load_mass", &rcs::hw::FR3Load::load_mass)
      .def_readwrite("f_x_cload", &rcs::hw::FR3Load::f_x_cload)
      .def_readwrite("load_inertia", &rcs::hw::FR3Load::load_inertia);

  py::enum_<rcs::hw::IKSolver>(hw, "IKSolver")
      .value("franka_ik", rcs::hw::IKSolver::franka_ik)
      .value("rcs_ik", rcs::hw::IKSolver::rcs_ik)
      .export_values();

  py::class_<rcs::hw::FR3Config, rcs::common::RobotConfig>(hw, "FR3Config")
      .def(py::init<>())
      .def_readwrite("ik_solver", &rcs::hw::FR3Config::ik_solver)
      .def_readwrite("speed_factor", &rcs::hw::FR3Config::speed_factor)
      .def_readwrite("load_parameters", &rcs::hw::FR3Config::load_parameters)
      .def_readwrite("nominal_end_effector_frame",
                     &rcs::hw::FR3Config::nominal_end_effector_frame)
      .def_readwrite("world_to_robot", &rcs::hw::FR3Config::world_to_robot)
      .def_readwrite("tcp_offset", &rcs::hw::FR3Config::tcp_offset)
      .def_readwrite("async_control", &rcs::hw::FR3Config::async_control);

  py::class_<rcs::hw::FHConfig, rcs::common::GripperConfig>(hw, "FHConfig")
      .def(py::init<>())
      .def_readwrite("grasping_width", &rcs::hw::FHConfig::grasping_width)
      .def_readwrite("speed", &rcs::hw::FHConfig::speed)
      .def_readwrite("force", &rcs::hw::FHConfig::force)
      .def_readwrite("epsilon_inner", &rcs::hw::FHConfig::epsilon_inner)
      .def_readwrite("epsilon_outer", &rcs::hw::FHConfig::epsilon_outer)
      .def_readwrite("async_control", &rcs::hw::FHConfig::async_control);

  py::class_<rcs::hw::FHState, rcs::common::GripperState>(hw, "FHState")
      .def(py::init<>())
      .def_readonly("width", &rcs::hw::FHState::width)
      .def_readonly("is_grasped", &rcs::hw::FHState::is_grasped)
      .def_readonly("is_moving", &rcs::hw::FHState::is_moving)
      .def_readonly("bool_state", &rcs::hw::FHState::bool_state)
      .def_readonly("last_commanded_width",
                    &rcs::hw::FHState::last_commanded_width)
      .def_readonly("max_unnormalized_width",
                    &rcs::hw::FHState::max_unnormalized_width)
      .def_readonly("temperature", &rcs::hw::FHState::temperature);

  py::class_<rcs::hw::FR3, rcs::common::Robot, std::shared_ptr<rcs::hw::FR3>>(
      hw, "FR3")
      .def(py::init<const std::string &,
                    std::optional<std::shared_ptr<rcs::common::IK>>>(),
           py::arg("ip"), py::arg("ik") = std::nullopt)
      .def("set_parameters", &rcs::hw::FR3::set_parameters, py::arg("cfg"))
      .def("get_parameters", &rcs::hw::FR3::get_parameters)
      .def("get_state", &rcs::hw::FR3::get_state)
      .def("set_default_robot_behavior",
           &rcs::hw::FR3::set_default_robot_behavior)
      .def("set_guiding_mode", &rcs::hw::FR3::set_guiding_mode,
           py::arg("x") = true, py::arg("y") = true, py::arg("z") = true,
           py::arg("roll") = true, py::arg("pitch") = true,
           py::arg("yaw") = true, py::arg("elbow") = true)
      .def("zero_torque_guiding", &rcs::hw::FR3::zero_torque_guiding)
      .def("osc_set_cartesian_position",
           &rcs::hw::FR3::osc_set_cartesian_position,
           py::arg("desired_pos_EE_in_base_frame"))
      .def("controller_set_joint_position",
           &rcs::hw::FR3::controller_set_joint_position, py::arg("desired_q"))
      .def("stop_control_thread", &rcs::hw::FR3::stop_control_thread)
      .def("automatic_error_recovery", &rcs::hw::FR3::automatic_error_recovery)
      .def("double_tap_robot_to_continue",
           &rcs::hw::FR3::double_tap_robot_to_continue)
      .def("set_cartesian_position_internal",
           &rcs::hw::FR3::set_cartesian_position_ik, py::arg("pose"))
      .def("set_cartesian_position_ik",
           &rcs::hw::FR3::set_cartesian_position_internal, py::arg("pose"),
           py::arg("max_time"), py::arg("elbow"), py::arg("max_force") = 5);

  py::class_<rcs::hw::FrankaHand, rcs::common::Gripper,
             std::shared_ptr<rcs::hw::FrankaHand>>(hw, "FrankaHand")
      .def(py::init<const std::string &, const rcs::hw::FHConfig &>(),
           py::arg("ip"), py::arg("cfg"))
      .def("get_parameters", &rcs::hw::FrankaHand::get_parameters)
      .def("get_state", &rcs::hw::FrankaHand::get_state)
      .def("set_parameters", &rcs::hw::FrankaHand::set_parameters,
           py::arg("cfg"))
      .def("is_grasped", &rcs::hw::FrankaHand::is_grasped)
      .def("homing", &rcs::hw::FrankaHand::homing);

  auto hw_except =
      hw.def_submodule("exceptions", "exceptions from the hardware module");
  py::register_exception<franka::Exception>(hw_except, "FrankaException",
                                            PyExc_RuntimeError);
  py::register_exception<franka::ModelException>(
      hw_except, "FrankaModelException", PyExc_RuntimeError);
  py::register_exception<franka::NetworkException>(
      hw_except, "FrankaNetworkException", PyExc_RuntimeError);
  py::register_exception<franka::ProtocolException>(
      hw_except, "FrankaProtocolException", PyExc_RuntimeError);
  py::register_exception<franka::IncompatibleVersionException>(
      hw_except, "FrankaIncompatibleVersionException", PyExc_RuntimeError);
  py::register_exception<franka::ControlException>(
      hw_except, "FrankaControlException", PyExc_RuntimeError);
  py::register_exception<franka::CommandException>(
      hw_except, "FrankaCommandException", PyExc_RuntimeError);
  py::register_exception<franka::RealtimeException>(
      hw_except, "FrankaRealtimeException", PyExc_RuntimeError);
  py::register_exception<franka::InvalidOperationException>(
      hw_except, "FrankaInvalidOperationException", PyExc_RuntimeError);
}
