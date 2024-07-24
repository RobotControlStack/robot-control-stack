#include <common/NRobotsWithGripper.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <common/utils.h>
#include <franka/exception.h>
#include <hw/FR3.h>
#include <hw/FrankaHand.h>
#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sim/FR3.h>
#include <sim/FrankaHand.h>
#include <sim/camera.h>

#include <memory>

#include "rl/mdl/UrdfFactory.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

/**
 * @brief Robot trampoline class for python bindings,
 * needed for pybind11 to override virtual functions,
 * see
 * https://pybind11.readthedocs.io/en/stable/advanced/classes.html
 */
class PyRobot : public rcs::common::Robot {
 public:
  using rcs::common::Robot::Robot;  // Inherit constructors

  rcs::common::RConfig *get_parameters() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RConfig *, rcs::common::Robot,
                           get_parameters, );
  }

  rcs::common::RState *get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RState *, rcs::common::Robot,
                           get_state, );
  }

  rcs::common::Pose get_cartesian_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           get_cartesian_position, );
  }

  void set_joint_position(const rcs::common::Vector7d &q) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, set_joint_position, q);
  }

  rcs::common::Vector7d get_joint_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Vector7d, rcs::common::Robot,
                           get_joint_position, );
  }

  void move_home() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, move_home, );
  }

  void reset() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, reset, );
  }

  void set_cartesian_position(const rcs::common::Pose &pose) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, set_cartesian_position,
                           pose);
  }

  rcs::common::Pose world_pose_to_robot_pose(
      const rcs::common::Pose &pose) override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           world_pose_to_robot_pose, pose);
  }

  rcs::common::Pose robot_pose_to_world_pose(
      const rcs::common::Pose &pose) override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           robot_pose_to_world_pose, pose);
  }
};

/**
 * @brief Gripper trampoline class for python bindings
 */
class PyGripper : public rcs::common::Gripper {
 public:
  using rcs::common::Gripper::Gripper;  // Inherit constructors

  rcs::common::GConfig *get_parameters() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GConfig *, rcs::common::Gripper,
                           get_parameters, );
  }

  rcs::common::GState *get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GState *, rcs::common::Gripper,
                           get_state, );
  }

  void set_normalized_width(double width, double force) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, set_normalized_width,
                           width, force);
  }

  double get_normalized_width() override {
    PYBIND11_OVERRIDE_PURE(bool, rcs::common::Gripper, get_normalized_width, );
  }

  bool is_grasped() override {
    PYBIND11_OVERRIDE_PURE(bool, rcs::common::Gripper, is_grasped, );
  }

  void grasp() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, grasp, );
  }

  void open() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, open, );
  }

  void shut() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, shut, );
  }
  void reset() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, reset, );
  }
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
      .def("as_vector", &rcs::common::RPY::as_vector)
      .def("is_close", &rcs::common::RPY::is_close, py::arg("other"),
           py::arg("eps") = 1e-8)
      .def("__str__", &rcs::common::RPY::str)
      .def(py::self + py::self)
      .def(py::pickle(
          [](const rcs::common::RPY &p) {  // dump
            return py::make_tuple(p.roll, p.pitch, p.yaw);
          },
          [](py::tuple t) {  // load
            return rcs::common::RPY(t[0].cast<double>(), t[1].cast<double>(),
                                    t[2].cast<double>());
          }));

  py::class_<rcs::common::Pose>(common, "Pose")
      .def(py::init<>())
      .def(py::init<const Eigen::Matrix4d &>(), py::arg("pose"))
      .def(py::init<const Eigen::Matrix3d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &, const Eigen::Vector3d &>(),
           py::arg("quaternion"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY &, const Eigen::Vector3d &>(),
           py::arg("rpy"), py::arg("translation"))
      .def(py::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(),
           py::arg("rpy_vector"), py::arg("translation"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rotation_rpy", &rcs::common::Pose::rotation_rpy)
      .def("xyzrpy", &rcs::common::Pose::xyzrpy)
      .def("interpolate", &rcs::common::Pose::interpolate, py::arg("dest_pose"),
           py::arg("progress"))
      .def("inverse", &rcs::common::Pose::inverse)
      .def("is_close", &rcs::common::Pose::is_close, py::arg("other"),
           py::arg("eps_r") = 1e-8, py::arg("eps_t") = 1e-8)
      .def("__str__", &rcs::common::Pose::str)
      .def(py::self * py::self)
      .def(py::pickle(
          [](const rcs::common::Pose &p) {  // dump
            return p.affine_array();
          },
          [](std::array<double, 16> t) {  // load
            return rcs::common::Pose(t);
          }));

  py::class_<rcs::common::RConfig>(common, "RConfig");
  py::class_<rcs::common::RState>(common, "RState");
  py::class_<rcs::common::GConfig>(common, "GConfig");
  py::class_<rcs::common::GState>(common, "GState");

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, PyRobot, std::shared_ptr<rcs::common::Robot>>(
      common, "Robot")
      .def("get_parameters", &rcs::common::Robot::get_parameters)
      .def("get_state", &rcs::common::Robot::get_state)
      .def("get_cartesian_position",
           &rcs::common::Robot::get_cartesian_position)
      .def("set_joint_position", &rcs::common::Robot::set_joint_position,
           py::arg("q"))
      .def("get_joint_position", &rcs::common::Robot::get_joint_position)
      .def("move_home", &rcs::common::Robot::move_home)
      .def("reset", &rcs::common::Robot::reset)
      .def("set_cartesian_position",
           &rcs::common::Robot::set_cartesian_position, py::arg("pose"));

  py::class_<rcs::common::Gripper, PyGripper,
             std::shared_ptr<rcs::common::Gripper>>(common, "Gripper")
      .def("get_parameters", &rcs::common::Gripper::get_parameters)
      .def("get_state", &rcs::common::Gripper::get_state)
      .def("set_normalized_width", &rcs::common::Gripper::set_normalized_width,
           py::arg("width"), py::arg("force") = 0)
      .def("get_normalized_width", &rcs::common::Gripper::get_normalized_width)
      .def("grasp", &rcs::common::Gripper::grasp)
      .def("is_grasped", &rcs::common::Gripper::is_grasped)
      .def("open", &rcs::common::Gripper::open)
      .def("shut", &rcs::common::Gripper::shut)
      .def("reset", &rcs::common::Gripper::reset);

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
      .def("get_parameters_g",
           &rcs::common::NRobotsWithGripper::get_parameters_g, py::arg("idxs"))
      .def("get_state_g", &rcs::common::NRobotsWithGripper::get_state_g,
           py::arg("idxs"))
      .def("grasp", &rcs::common::NRobotsWithGripper::grasp, py::arg("idxs"))
      .def("open", &rcs::common::NRobotsWithGripper::open, py::arg("idxs"))
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
                     &rcs::hw::FR3Config::nominal_end_effector_frame)
      .def_readwrite("world_to_robot", &rcs::hw::FR3Config::world_to_robot);

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
      .def_readonly("last_commanded_width",
                    &rcs::hw::FHState::last_commanded_width)
      .def_readonly("max_unnormalized_width",
                    &rcs::hw::FHState::max_unnormalized_width)
      .def_readonly("temperature", &rcs::hw::FHState::temperature);

  py::class_<rcs::hw::FR3, rcs::common::Robot, std::shared_ptr<rcs::hw::FR3>>(
      hw, "FR3")
      .def(py::init<const std::string &, const std::optional<std::string> &>(),
           py::arg("ip"), py::arg("filename") = std::nullopt)
      .def("set_parameters", &rcs::hw::FR3::set_parameters, py::arg("cfg"))
      .def("get_parameters", &rcs::hw::FR3::get_parameters)
      .def("get_state", &rcs::hw::FR3::get_state)
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

  // SIM MODULE
  auto sim = m.def_submodule("sim", "sim module");
  py::class_<rcs::sim::FR3Config, rcs::common::RConfig>(sim, "FR3Config")
      .def(py::init<>())
      .def_readwrite("tcp_offset", &rcs::sim::FR3Config::tcp_offset)
      .def_readwrite("joint_rotational_tolerance",
                     &rcs::sim::FR3Config::joint_rotational_tolerance)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::FR3Config::seconds_between_callbacks)
      .def_readwrite("ik_duration_in_milliseconds",
                     &rcs::sim::FR3Config::ik_duration_in_milliseconds)
      .def_readwrite("realtime", &rcs::sim::FR3Config::realtime)
      .def_readwrite("trajectory_trace",
                     &rcs::sim::FR3Config::trajectory_trace);
  py::class_<rcs::sim::FR3State, rcs::common::RState>(sim, "FR3State")
      .def(py::init<>())
      .def_readonly("previous_angles", &rcs::sim::FR3State::previous_angles)
      .def_readonly("target_angles", &rcs::sim::FR3State::target_angles)
      .def_readonly("inverse_tcp_offset",
                    &rcs::sim::FR3State::inverse_tcp_offset)
      .def_readonly("ik_success", &rcs::sim::FR3State::ik_success)
      .def_readonly("collision", &rcs::sim::FR3State::collision)
      .def_readonly("is_moving", &rcs::sim::FR3State::is_moving)
      .def_readonly("is_arrived", &rcs::sim::FR3State::is_arrived);
  py::class_<rcs::sim::FHConfig, rcs::common::GConfig>(sim, "FHConfig")
      .def(py::init<>())
      .def_readwrite("epsilon_inner", &rcs::sim::FHConfig::epsilon_inner)
      .def_readwrite("epsilon_outer", &rcs::sim::FHConfig::epsilon_outer);
  py::class_<rcs::sim::FHState, rcs::common::GState>(sim, "FHState")
      .def(py::init<>())
      .def_readonly("last_commanded_width",
                    &rcs::sim::FHState::last_commanded_width)
      .def_readonly("max_unnormalized_width",
                    &rcs::sim::FHState::max_unnormalized_width);

  py::class_<rcs::sim::Sim, std::shared_ptr<rcs::sim::Sim>>(sim, "Sim")
      .def(py::init([](long m, long d) {
             return std::make_shared<rcs::sim::Sim>((mjModel *)m, (mjData *)d);
           }),
           py::arg("mjmdl"), py::arg("mjdata"))
      .def("step_until_convergence", &rcs::sim::Sim::step_until_convergence)
      .def("step", &rcs::sim::Sim::step, py::arg("k"))
      .def("reset", &rcs::sim::Sim::reset);
  py::class_<rcs::sim::FrankaHand, rcs::common::Gripper,
             std::shared_ptr<rcs::sim::FrankaHand>>(sim, "FrankaHand")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>, const std::string &,
                    const rcs::sim::FHConfig &>(),
           py::arg("sim"), py::arg("id"), py::arg("cfg"))
      .def("get_parameters", &rcs::sim::FrankaHand::get_parameters)
      .def("get_state", &rcs::sim::FrankaHand::get_state)
      .def("set_parameters", &rcs::sim::FrankaHand::set_parameters,
           py::arg("cfg"));
  py::class_<rcs::sim::FR3, rcs::common::Robot, std::shared_ptr<rcs::sim::FR3>>(
      sim, "FR3")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>, const std::string &,
                    const std::string &>(),
           py::arg("sim"), py::arg("id"), py::arg("rlmdl"))
      .def("get_parameters", &rcs::sim::FR3::get_parameters)
      .def("set_parameters", &rcs::sim::FR3::set_parameters, py::arg("cfg"))
      .def("get_state", &rcs::sim::FR3::get_state);
  py::enum_<rcs::sim::CameraType>(sim, "CameraType")
      .value("free", rcs::sim::CameraType::free)
      .value("tracking", rcs::sim::CameraType::tracking)
      .value("fixed", rcs::sim::CameraType::fixed)
      .value("default_free", rcs::sim::CameraType::default_free)
      .export_values();
  py::class_<rcs::sim::SimCameraConfig>(sim, "SimCameraConfig")
      .def(py::init<>())
      .def_readwrite("identifier", &rcs::sim::SimCameraConfig::identifier)
      .def_readwrite("type", &rcs::sim::SimCameraConfig::type)
      .def_readwrite("on_screen_render",
                     &rcs::sim::SimCameraConfig::on_screen_render);
  py::class_<rcs::sim::SimCameraSetConfig>(sim, "SimCameraSetConfig")
      .def(py::init<>())
      .def_readwrite("cameras", &rcs::sim::SimCameraSetConfig::cameras)
      .def_readwrite("frame_rate", &rcs::sim::SimCameraSetConfig::frame_rate)
      .def_readwrite("resolution_width",
                     &rcs::sim::SimCameraSetConfig::resolution_width)
      .def_readwrite("resolution_height",
                     &rcs::sim::SimCameraSetConfig::resolution_height);
  py::class_<rcs::sim::FrameSet>(sim, "FrameSet")
      .def(py::init<>())
      .def_readonly("color_frames", &rcs::sim::FrameSet::color_frames)
      .def_readonly("timestamp", &rcs::sim::FrameSet::timestamp);
  py::class_<rcs::sim::SimCameraSet>(sim, "SimCameraSet")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    rcs::sim::SimCameraSetConfig>(),
           py::arg("sim"), py::arg("cfg"))
      .def("buffer_size", &rcs::sim::SimCameraSet::buffer_size)
      .def("clear_buffer", &rcs::sim::SimCameraSet::clear_buffer)
      .def("get_latest_frameset", &rcs::sim::SimCameraSet::get_latest_frameset)
      .def("get_timestamp_frameset",
           &rcs::sim::SimCameraSet::get_timestamp_frameset, py::arg("ts"));
}
