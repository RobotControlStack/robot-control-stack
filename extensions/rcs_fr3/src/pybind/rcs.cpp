#include <franka/exception.h>
#include <franka/robot_state.h>
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

  py::enum_<franka::RobotMode>(hw, "RobotMode")
      .value("kOther", franka::RobotMode::kOther)
      .value("kIdle", franka::RobotMode::kIdle)
      .value("kMove", franka::RobotMode::kMove)
      .value("kGuiding", franka::RobotMode::kGuiding)
      .value("kReflex", franka::RobotMode::kReflex)
      .value("kUserStopped", franka::RobotMode::kUserStopped)
      .value("kAutomaticErrorRecovery",
             franka::RobotMode::kAutomaticErrorRecovery)
      .export_values();

  py::class_<franka::RobotState>(hw, "RobotState")
      .def(py::init<>())
      .def_readonly("O_T_EE", &franka::RobotState::O_T_EE)
      .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d)
      .def_readonly("F_T_EE", &franka::RobotState::F_T_EE)
      .def_readonly("F_T_NE", &franka::RobotState::F_T_NE)
      .def_readonly("NE_T_EE", &franka::RobotState::NE_T_EE)
      .def_readonly("EE_T_K", &franka::RobotState::EE_T_K)
      .def_readonly("m_ee", &franka::RobotState::m_ee)
      .def_readonly("I_ee", &franka::RobotState::I_ee)
      .def_readonly("F_x_Cee", &franka::RobotState::F_x_Cee)
      .def_readonly("m_load", &franka::RobotState::m_load)
      .def_readonly("I_load", &franka::RobotState::I_load)
      .def_readonly("F_x_Cload", &franka::RobotState::F_x_Cload)
      .def_readonly("m_total", &franka::RobotState::m_total)
      .def_readonly("I_total", &franka::RobotState::I_total)
      .def_readonly("F_x_Ctotal", &franka::RobotState::F_x_Ctotal)
      .def_readonly("elbow", &franka::RobotState::elbow)
      .def_readonly("elbow_d", &franka::RobotState::elbow_d)
      .def_readonly("elbow_c", &franka::RobotState::elbow_c)
      .def_readonly("delbow_c", &franka::RobotState::delbow_c)
      .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c)
      .def_readonly("tau_J", &franka::RobotState::tau_J)
      .def_readonly("tau_J_d", &franka::RobotState::tau_J_d)
      .def_readonly("dtau_J", &franka::RobotState::dtau_J)
      .def_readonly("q", &franka::RobotState::q)
      .def_readonly("q_d", &franka::RobotState::q_d)
      .def_readonly("dq", &franka::RobotState::dq)
      .def_readonly("dq_d", &franka::RobotState::dq_d)
      .def_readonly("ddq_d", &franka::RobotState::ddq_d)
      .def_readonly("joint_contact", &franka::RobotState::joint_contact)
      .def_readonly("cartesian_contact", &franka::RobotState::cartesian_contact)
      .def_readonly("joint_collision", &franka::RobotState::joint_collision)
      .def_readonly("cartesian_collision",
                    &franka::RobotState::cartesian_collision)
      .def_readonly("tau_ext_hat_filtered",
                    &franka::RobotState::tau_ext_hat_filtered)
      .def_readonly("O_F_ext_hat_K", &franka::RobotState::O_F_ext_hat_K)
      .def_readonly("K_F_ext_hat_K", &franka::RobotState::K_F_ext_hat_K)
      .def_readonly("O_dP_EE_d", &franka::RobotState::O_dP_EE_d)
      .def_readonly("O_ddP_O", &franka::RobotState::O_ddP_O)
      .def_readonly("O_T_EE_c", &franka::RobotState::O_T_EE_c)
      .def_readonly("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
      .def_readonly("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
      .def_readonly("theta", &franka::RobotState::theta)
      .def_readonly("dtheta", &franka::RobotState::dtheta)
      //   .def_readonly("current_errors", &franka::RobotState::current_errors)
      //   .def_readonly("last_motion_errors",
      //                 &franka::RobotState::last_motion_errors)
      .def_readonly("control_command_success_rate",
                    &franka::RobotState::control_command_success_rate)
      //   .def_readonly("time", &franka::RobotState::time)
      .def_readonly("robot_mode", &franka::RobotState::robot_mode);

  py::object robot_state =
      (py::object)py::module_::import("rcs").attr("common").attr("RobotState");
  py::class_<rcs::hw::FrankaState>(hw, "FrankaState", robot_state)
      .def(py::init<>())
      .def_readonly("robot_state", &rcs::hw::FrankaState::robot_state);
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
      // the robot specific subclasses (FR3Config, PandaConfig) are defined in
      // Python, they fill in the values from rcs.ROBOTS
      .def(py::init<>())
      .def_readwrite("ik_solver", &rcs::hw::FrankaConfig::ik_solver)
      .def_readwrite("speed_factor", &rcs::hw::FrankaConfig::speed_factor)
      .def_readwrite("policy_rate", &rcs::hw::FrankaConfig::policy_rate)
      .def_readwrite("timestep", &rcs::hw::FrankaConfig::timestep)
      .def_readwrite("kp", &rcs::hw::FrankaConfig::kp)
      .def_readwrite("kd", &rcs::hw::FrankaConfig::kd)
      .def_readwrite("allow_high_collision",
                     &rcs::hw::FrankaConfig::allow_high_collision)
      .def_readwrite("kp_p", &rcs::hw::FrankaConfig::kp_p)
      .def_readwrite("kp_r", &rcs::hw::FrankaConfig::kp_r)
      .def_readwrite("load_parameters", &rcs::hw::FrankaConfig::load_parameters)
      .def_readwrite("world_to_robot", &rcs::hw::FrankaConfig::world_to_robot)
      .def_property(
          "tcp_offset",
          [](const rcs::hw::FrankaConfig& config) { return config.tcp_offset; },
          [](rcs::hw::FrankaConfig& config,
             const rcs::common::Pose& tcp_offset) {
            config.tcp_offset = tcp_offset;
            config.tcp_offset_explicit = true;
          })
      .def_readwrite("tcp_offset_explicit",
                     &rcs::hw::FrankaConfig::tcp_offset_explicit)
      .def_readwrite("async_control", &rcs::hw::FrankaConfig::async_control)
      .def_readwrite("blocking_move_on_start",
                     &rcs::hw::FrankaConfig::blocking_move_on_start)
      .def_readwrite("approach_joint_speed",
                     &rcs::hw::FrankaConfig::approach_joint_speed)
      .def_readwrite("approach_cartesian_speed",
                     &rcs::hw::FrankaConfig::approach_cartesian_speed)
      .def_readwrite("approach_rotation_speed",
                     &rcs::hw::FrankaConfig::approach_rotation_speed)
      .def_readwrite("ignore_realtime", &rcs::hw::FrankaConfig::ignore_realtime)
      .def_readwrite("ip", &rcs::hw::FrankaConfig::ip);

  py::object gripper_config =
      (py::object)py::module_::import("rcs").attr("common").attr(
          "GripperConfig");
  rcs::hw::FHConfig default_gripper_config;
  py::class_<rcs::hw::FHConfig>(hw, "FHConfig", gripper_config)
      .def(py::init([](const std::string& ip, double grasping_width,
                       double speed, double force, double epsilon_inner,
                       double epsilon_outer, bool async_control) {
             rcs::hw::FHConfig cfg;
             cfg.ip = ip;
             cfg.grasping_width = grasping_width;
             cfg.speed = speed;
             cfg.force = force;
             cfg.epsilon_inner = epsilon_inner;
             cfg.epsilon_outer = epsilon_outer;
             cfg.async_control = async_control;
             return cfg;
           }),
           py::arg("ip"),
           py::arg("grasping_width") = default_gripper_config.grasping_width,
           py::arg("speed") = default_gripper_config.speed,
           py::arg("force") = default_gripper_config.force,
           py::arg("epsilon_inner") = default_gripper_config.epsilon_inner,
           py::arg("epsilon_outer") = default_gripper_config.epsilon_outer,
           py::arg("async_control") = default_gripper_config.async_control)
      .def_readwrite("ip", &rcs::hw::FHConfig::ip)
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
      .def(py::init<const rcs::hw::FrankaConfig&,
                    std::optional<std::shared_ptr<rcs::common::Kinematics>>>(),
           py::arg("cfg"), py::arg("ik") = std::nullopt)
      .def("set_config", &rcs::hw::Franka::set_config, py::arg("cfg"))
      .def("get_config", &rcs::hw::Franka::get_config)
      .def("get_state", &rcs::hw::Franka::get_state)
      .def("get_cartesian_flange_position",
           &rcs::hw::Franka::get_cartesian_flange_position)
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
      .def(py::init<const rcs::hw::FHConfig&>(), py::arg("cfg"))
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
