#include <franka/exception.h>
#include <hw/Franka.h>
#include <hw/FrankaHand.h>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "rcs/Kinematics.h"
#include "rcs/Pose.h"
#include "rcs/Robot.h"
#include "rcs/utils.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = R"pbdoc(
        Franka Python Bindings
        ----------------------

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
  auto hw = m.def_submodule("hw", "rcs franka module");

  py::object robot_state =
      (py::object)py::module_::import("rcs").attr("common").attr("RobotState");
  py::class_<rcs::hw::FrankaState>(hw, "FrankaState", robot_state)
      .def(py::init<>());
  py::class_<rcs::hw::FrankaLoad>(hw, "FrankaLoad")
      .def(py::init<>())
      .def_readwrite("load_mass", &rcs::hw::FrankaLoad::load_mass)
      .def_readwrite("f_x_cload", &rcs::hw::FrankaLoad::f_x_cload)
      .def_readwrite("load_inertia", &rcs::hw::FrankaLoad::load_inertia);

  py::enum_<rcs::hw::IKSolver>(hw, "IKSolver")
      .value("franka_ik", rcs::hw::IKSolver::franka_ik)
      .value("rcs_ik", rcs::hw::IKSolver::rcs_ik)
      .export_values();

  py::object robot_config =
      (py::object)py::module_::import("rcs").attr("common").attr("RobotConfig");
  py::class_<rcs::hw::FrankaConfig>(hw, "FrankaConfig", robot_config)
      .def(py::init<>())
      .def_readwrite("ik_solver", &rcs::hw::FrankaConfig::ik_solver)
      .def_readwrite("speed_factor", &rcs::hw::FrankaConfig::speed_factor)
      .def_readwrite("load_parameters", &rcs::hw::FrankaConfig::load_parameters)
      .def_readwrite("nominal_end_effector_frame",
                     &rcs::hw::FrankaConfig::nominal_end_effector_frame)
      .def_readwrite("world_to_robot", &rcs::hw::FrankaConfig::world_to_robot)
      .def_readwrite("async_control", &rcs::hw::FrankaConfig::async_control);

  py::class_<rcs::hw::FR3Config, rcs::hw::FrankaConfig>(hw, "FR3Config")
      .def(py::init<>());
  py::class_<rcs::hw::PandaConfig, rcs::hw::FrankaConfig>(hw, "PandaConfig")
      .def(py::init<>());

  py::object gripper_config =
      (py::object)py::module_::import("rcs").attr("common").attr(
          "GripperConfig");
  py::class_<rcs::hw::FHConfig>(hw, "FHConfig", gripper_config)
      .def(py::init<>())
      .def_readwrite("grasping_width", &rcs::hw::FHConfig::grasping_width)
      .def_readwrite("speed", &rcs::hw::FHConfig::speed)
      .def_readwrite("force", &rcs::hw::FHConfig::force)
      .def_readwrite("epsilon_inner", &rcs::hw::FHConfig::epsilon_inner)
      .def_readwrite("epsilon_outer", &rcs::hw::FHConfig::epsilon_outer)
      .def_readwrite("async_control", &rcs::hw::FHConfig::async_control);

  py::object gripper_state =
      (py::object)py::module_::import("rcs").attr("common").attr(
          "GripperState");
  py::class_<rcs::hw::FHState>(hw, "FHState", gripper_state)
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

  py::object robot =
      (py::object)py::module_::import("rcs").attr("common").attr("Robot");
  py::class_<rcs::hw::Franka, std::shared_ptr<rcs::hw::Franka>>(hw, "Franka",
                                                                robot)
      .def(py::init<const std::string&,
                    std::optional<std::shared_ptr<rcs::common::Kinematics>>>(),
           py::arg("ip"), py::arg("ik") = std::nullopt)
      .def("set_config", &rcs::hw::Franka::set_config, py::arg("cfg"))
      .def("get_config", &rcs::hw::Franka::get_config)
      .def("get_state", &rcs::hw::Franka::get_state)
      .def("set_default_robot_behavior",
           &rcs::hw::Franka::set_default_robot_behavior)
      .def("set_guiding_mode", &rcs::hw::Franka::set_guiding_mode,
           py::arg("x") = true, py::arg("y") = true, py::arg("z") = true,
           py::arg("roll") = true, py::arg("pitch") = true,
           py::arg("yaw") = true, py::arg("elbow") = true)
      .def("zero_torque_guiding", &rcs::hw::Franka::zero_torque_guiding)
      .def("osc_set_cartesian_position",
           &rcs::hw::Franka::osc_set_cartesian_position,
           py::arg("desired_pos_EE_in_base_frame"))
      .def("controller_set_joint_position",
           &rcs::hw::Franka::controller_set_joint_position,
           py::arg("desired_q"))
      .def("stop_control_thread", &rcs::hw::Franka::stop_control_thread)
      .def("automatic_error_recovery",
           &rcs::hw::Franka::automatic_error_recovery)
      .def("double_tap_robot_to_continue",
           &rcs::hw::Franka::double_tap_robot_to_continue)
      .def("set_cartesian_position_internal",
           &rcs::hw::Franka::set_cartesian_position_ik, py::arg("pose"))
      .def("set_cartesian_position_ik",
           &rcs::hw::Franka::set_cartesian_position_internal, py::arg("pose"),
           py::arg("max_time"), py::arg("elbow"), py::arg("max_force") = 5);

  py::object gripper =
      (py::object)py::module_::import("rcs").attr("common").attr("Gripper");
  py::class_<rcs::hw::FrankaHand, std::shared_ptr<rcs::hw::FrankaHand>>(
      hw, "FrankaHand", gripper)
      .def(py::init<const std::string&, const rcs::hw::FHConfig&>(),
           py::arg("ip"), py::arg("cfg"))
      .def("get_config", &rcs::hw::FrankaHand::get_config)
      .def("get_state", &rcs::hw::FrankaHand::get_state)
      .def("set_config", &rcs::hw::FrankaHand::set_config, py::arg("cfg"))
      .def("is_grasped", &rcs::hw::FrankaHand::is_grasped)
      .def("homing", &rcs::hw::FrankaHand::homing)
      .def("close", &rcs::hw::FrankaHand::close);

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
