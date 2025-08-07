#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sim/SimGripper.h>
#include <sim/SimTilburgHand.h>
#include <sim/SimRobot.h>
#include <sim/camera.h>
#include <sim/gui.h>

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

/**
 * @brief Robot trampoline class for python bindings,
 * needed for pybind11 to override virtual functions,
 * see
 * https://pybind11.readthedocs.io/en/stable/advanced/classes.html
 */
class PyRobot : public rcs::common::Robot {
 public:
  using rcs::common::Robot::Robot;  // Inherit constructors

  rcs::common::RobotConfig *get_parameters() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RobotConfig *, rcs::common::Robot,
                           get_parameters, );
  }

  rcs::common::RobotState *get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RobotState *, rcs::common::Robot,
                           get_state, );
  }

  rcs::common::Pose get_cartesian_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           get_cartesian_position, );
  }

  void set_joint_position(const rcs::common::VectorXd &q) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, set_joint_position, q);
  }

  rcs::common::VectorXd get_joint_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::VectorXd, rcs::common::Robot,
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
};

/**
 * @brief Gripper trampoline class for python bindings
 */
class PyGripper : public rcs::common::Gripper {
 public:
  using rcs::common::Gripper::Gripper;  // Inherit constructors

  rcs::common::GripperConfig *get_parameters() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GripperConfig *, rcs::common::Gripper,
                           get_parameters, );
  }

  rcs::common::GripperState *get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GripperState *, rcs::common::Gripper,
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

/**
 * @brief Hand trampoline class for python bindings
 */
class PyHand : public rcs::common::Hand {
 public:
  using rcs::common::Hand::Hand;  // Inherit constructors

  rcs::common::HandConfig *get_parameters() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::HandConfig *, rcs::common::Hand,
                           get_parameters, );
  }

  rcs::common::HandState *get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::HandState *, rcs::common::Hand,
                           get_state, );
  }

  void set_normalized_joint_poses(const rcs::common::VectorXd &q) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, set_normalized_joint_poses,
                           q);
  }

  rcs::common::VectorXd get_normalized_joint_poses() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::VectorXd, rcs::common::Hand, get_normalized_joint_poses, );
  }

  bool is_grasped() override {
    PYBIND11_OVERRIDE_PURE(bool, rcs::common::Hand, is_grasped, );
  }

  void grasp() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, grasp, );
  }

  void open() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, open, );
  }

  void shut() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, shut, );
  }
  void reset() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, reset, );
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

  common.def("_bootstrap_egl", &rcs::common::bootstrap_egl, py::arg("fn_addr"),
             py::arg("display"), py::arg("context"));
  common.def("IdentityTranslation", &rcs::common::IdentityTranslation);
  common.def("IdentityRotMatrix", &rcs::common::IdentityRotMatrix);
  common.def("IdentityRotQuatVec", &rcs::common::IdentityRotQuatVec);
  common.def("FrankaHandTCPOffset", &rcs::common::FrankaHandTCPOffset);

  py::class_<rcs::common::BaseCameraConfig>(common, "BaseCameraConfig")
      .def(py::init<const std::string &, int, int, int>(),
           py::arg("identifier"), py::arg("frame_rate"),
           py::arg("resolution_width"), py::arg("resolution_height"))
      .def_readwrite("identifier", &rcs::common::BaseCameraConfig::identifier)
      .def_readwrite("frame_rate", &rcs::common::BaseCameraConfig::frame_rate)
      .def_readwrite("resolution_width",
                     &rcs::common::BaseCameraConfig::resolution_width)
      .def_readwrite("resolution_height",
                     &rcs::common::BaseCameraConfig::resolution_height);

  py::class_<rcs::common::RPY>(common, "RPY")
      .def(py::init<double, double, double>(), py::arg("roll") = 0.0,
           py::arg("pitch") = 0.0, py::arg("yaw") = 0.0)
      .def(py::init<Eigen::Vector3d>(), py::arg("rpy"))
      .def_readwrite("roll", &rcs::common::RPY::roll)
      .def_readwrite("pitch", &rcs::common::RPY::pitch)
      .def_readwrite("yaw", &rcs::common::RPY::yaw)
      .def("rotation_matrix", &rcs::common::RPY::rotation_matrix)
      .def("as_vector", &rcs::common::RPY::as_vector)
      .def("as_quaternion_vector", &rcs::common::RPY::as_quaternion_vector)
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
      .def(py::init<const Eigen::Matrix4d &>(), py::arg("pose_matrix"))
      .def(py::init<const Eigen::Matrix3d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &, const Eigen::Vector3d &>(),
           py::arg("quaternion"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY &, const Eigen::Vector3d &>(),
           py::arg("rpy"), py::arg("translation"))
      .def(py::init<const Eigen::Vector3d &, const Eigen::Vector3d &>(),
           py::arg("rpy_vector"), py::arg("translation"))
      .def(py::init<const Eigen::Vector3d &>(), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &>(), py::arg("quaternion"))
      .def(py::init<const rcs::common::RPY &>(), py::arg("rpy"))
      .def(py::init<const Eigen::Matrix3d &>(), py::arg("rotation"))
      .def(py::init<const rcs::common::Pose &>(), py::arg("pose"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rotation_rpy", &rcs::common::Pose::rotation_rpy)
      .def("xyzrpy", &rcs::common::Pose::xyzrpy)
      .def("interpolate", &rcs::common::Pose::interpolate, py::arg("dest_pose"),
           py::arg("progress"))
      .def("inverse", &rcs::common::Pose::inverse)
      .def("total_angle", &rcs::common::Pose::total_angle)
      .def("limit_rotation_angle", &rcs::common::Pose::limit_rotation_angle,
           py::arg("max_angle"))
      .def("limit_translation_length",
           &rcs::common::Pose::limit_translation_length, py::arg("max_length"))
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

  py::class_<rcs::common::IK, std::shared_ptr<rcs::common::IK>>(common, "IK")
      .def("ik", &rcs::common::IK::ik, py::arg("pose"), py::arg("q0"),
           py::arg("tcp_offset") = rcs::common::Pose::Identity())
      .def("forward", &rcs::common::IK::forward, py::arg("q0"),
           py::arg("tcp_offset") = rcs::common::Pose::Identity());

  py::class_<rcs::common::RL, rcs::common::IK,
             std::shared_ptr<rcs::common::RL>>(common, "RL")
      .def(py::init<const std::string &, size_t>(), py::arg("urdf_path"),
           py::arg("max_duration_ms") = 300);

  py::class_<rcs::common::Pin, rcs::common::IK,
             std::shared_ptr<rcs::common::Pin>>(common, "Pin")
      .def(py::init<const std::string &, const std::string &, bool>(),
           py::arg("path"), py::arg("frame_id") = "fr3_link8",
           py::arg("urdf") = true);

  py::enum_<rcs::common::RobotType>(common, "RobotType")
      .value("FR3", rcs::common::RobotType::FR3)
      .value("UR5e", rcs::common::RobotType::UR5e)
      .value("SO101", rcs::common::RobotType::SO101)
      .export_values();

  py::enum_<rcs::common::RobotPlatform>(common, "RobotPlatform")
      .value("HARDWARE", rcs::common::RobotPlatform::HARDWARE)
      .value("SIMULATION", rcs::common::RobotPlatform::SIMULATION)
      .export_values();

  py::class_<rcs::common::RobotMetaConfig>(common, "RobotMetaConfig")
      .def_readonly("q_home", &rcs::common::RobotMetaConfig::q_home)
      .def_readonly("dof", &rcs::common::RobotMetaConfig::dof)
      .def_readonly("joint_limits",
                    &rcs::common::RobotMetaConfig::joint_limits);

  common.def(
      "robots_meta_config",
      [](rcs::common::RobotType robot_type) -> rcs::common::RobotMetaConfig {
        return rcs::common::robots_meta_config.at(robot_type);
      },
      py::arg("robot_type"));

  py::class_<rcs::common::RobotConfig>(common, "RobotConfig")
      .def(py::init<>())
      .def_readwrite("robot_type", &rcs::common::RobotConfig::robot_type)
      .def_readwrite("robot_platform",
                     &rcs::common::RobotConfig::robot_platform);
  py::class_<rcs::common::RobotState>(common, "RobotState");
  py::class_<rcs::common::GripperConfig>(common, "GripperConfig");
  py::class_<rcs::common::GripperState>(common, "GripperState");
  py::enum_<rcs::common::GraspType>(common, "GraspType")
      .value("POWER_GRASP", rcs::common::GraspType::POWER_GRASP)
      .value("PRECISION_GRASP", rcs::common::GraspType::PRECISION_GRASP)
      .value("LATERAL_GRASP", rcs::common::GraspType::LATERAL_GRASP)
      .value("TRIPOD_GRASP", rcs::common::GraspType::TRIPOD_GRASP)
      .export_values();
  py::class_<rcs::common::HandConfig>(common, "HandConfig")
      .def(py::init<>());
  py::class_<rcs::common::HandState>(common, "HandState")
      .def(py::init<>());

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, PyRobot, std::shared_ptr<rcs::common::Robot>>(
      common, "Robot")
      .def("get_parameters", &rcs::common::Robot::get_parameters)
      .def("get_state", &rcs::common::Robot::get_state)
      .def("get_cartesian_position",
           &rcs::common::Robot::get_cartesian_position)
      .def("set_joint_position", &rcs::common::Robot::set_joint_position,
           py::arg("q"), py::call_guard<py::gil_scoped_release>())
      .def("get_joint_position", &rcs::common::Robot::get_joint_position)
      .def("move_home", &rcs::common::Robot::move_home,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Robot::reset)
      .def("set_cartesian_position",
           &rcs::common::Robot::set_cartesian_position, py::arg("pose"),
           py::call_guard<py::gil_scoped_release>())
      .def("get_ik", &rcs::common::Robot::get_ik)
      .def("get_base_pose_in_world_coordinates",
           &rcs::common::Robot::get_base_pose_in_world_coordinates)
      .def("to_pose_in_robot_coordinates",
           &rcs::common::Robot::to_pose_in_robot_coordinates,
           py::arg("pose_in_world_coordinates"))
      .def("to_pose_in_world_coordinates",
           &rcs::common::Robot::to_pose_in_world_coordinates,
           py::arg("pose_in_robot_coordinates"));

  py::class_<rcs::common::Gripper, PyGripper,
             std::shared_ptr<rcs::common::Gripper>>(common, "Gripper")
      .def("get_parameters", &rcs::common::Gripper::get_parameters)
      .def("get_state", &rcs::common::Gripper::get_state)
      .def("set_normalized_width", &rcs::common::Gripper::set_normalized_width,
           py::arg("width"), py::arg("force") = 0)
      .def("get_normalized_width", &rcs::common::Gripper::get_normalized_width)
      .def("grasp", &rcs::common::Gripper::grasp,
           py::call_guard<py::gil_scoped_release>())
      .def("is_grasped", &rcs::common::Gripper::is_grasped)
      .def("open", &rcs::common::Gripper::open,
           py::call_guard<py::gil_scoped_release>())
      .def("shut", &rcs::common::Gripper::shut,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Gripper::reset,
           py::call_guard<py::gil_scoped_release>());

  py::class_<rcs::common::Hand, PyHand,
             std::shared_ptr<rcs::common::Hand>>(common, "Hand")
      .def(py::init<>())
      .def("get_parameters", &rcs::common::Hand::get_parameters)
      .def("get_state", &rcs::common::Hand::get_state)
      .def("set_normalized_joint_poses", &rcs::common::Hand::set_normalized_joint_poses,
           py::arg("q"))
      .def("get_normalized_joint_poses", &rcs::common::Hand::get_normalized_joint_poses)
      .def("grasp", &rcs::common::Hand::grasp,
           py::call_guard<py::gil_scoped_release>())
      .def("is_grasped", &rcs::common::Hand::is_grasped)
      .def("open", &rcs::common::Hand::open,
           py::call_guard<py::gil_scoped_release>())
      .def("shut", &rcs::common::Hand::shut,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Hand::reset,
           py::call_guard<py::gil_scoped_release>());

  // SIM MODULE
  auto sim = m.def_submodule("sim", "sim module");
  py::class_<rcs::sim::SimRobotConfig, rcs::common::RobotConfig>(
      sim, "SimRobotConfig")
      .def(py::init<>())
      .def_readwrite("tcp_offset", &rcs::sim::SimRobotConfig::tcp_offset)
      .def_readwrite("joint_rotational_tolerance",
                     &rcs::sim::SimRobotConfig::joint_rotational_tolerance)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::SimRobotConfig::seconds_between_callbacks)
      .def_readwrite("realtime", &rcs::sim::SimRobotConfig::realtime)
      .def_readwrite("trajectory_trace",
                     &rcs::sim::SimRobotConfig::trajectory_trace)
      .def_readwrite("robot_type", &rcs::sim::SimRobotConfig::robot_type)
      .def_readwrite("arm_collision_geoms",
                     &rcs::sim::SimRobotConfig::arm_collision_geoms)
      .def_readwrite("joints", &rcs::sim::SimRobotConfig::joints)
      .def_readwrite("attachment_site",
                     &rcs::sim::SimRobotConfig::attachment_site)
      .def_readwrite("actuators", &rcs::sim::SimRobotConfig::actuators)
      .def_readwrite("base", &rcs::sim::SimRobotConfig::base)
      .def("add_id", &rcs::sim::SimRobotConfig::add_id, py::arg("id"));
  py::class_<rcs::sim::SimRobotState, rcs::common::RobotState>(sim,
                                                               "SimRobotState")
      .def(py::init<>())
      .def_readonly("previous_angles",
                    &rcs::sim::SimRobotState::previous_angles)
      .def_readonly("target_angles", &rcs::sim::SimRobotState::target_angles)
      .def_readonly("inverse_tcp_offset",
                    &rcs::sim::SimRobotState::inverse_tcp_offset)
      .def_readonly("ik_success", &rcs::sim::SimRobotState::ik_success)
      .def_readonly("collision", &rcs::sim::SimRobotState::collision)
      .def_readonly("is_moving", &rcs::sim::SimRobotState::is_moving)
      .def_readonly("is_arrived", &rcs::sim::SimRobotState::is_arrived);
  py::class_<rcs::sim::SimGripperConfig, rcs::common::GripperConfig>(
      sim, "SimGripperConfig")
      .def(py::init<>())
      .def_readwrite("epsilon_inner",
                     &rcs::sim::SimGripperConfig::epsilon_inner)
      .def_readwrite("epsilon_outer",
                     &rcs::sim::SimGripperConfig::epsilon_outer)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::SimGripperConfig::seconds_between_callbacks)
      .def_readwrite("ignored_collision_geoms",
                     &rcs::sim::SimGripperConfig::ignored_collision_geoms)
      .def_readwrite("collision_geoms",
                     &rcs::sim::SimGripperConfig::collision_geoms)
      .def_readwrite("collision_geoms_fingers",
                     &rcs::sim::SimGripperConfig::collision_geoms_fingers)
      .def_readwrite("joint", &rcs::sim::SimGripperConfig::joint)
      .def_readwrite("max_joint_width",
                     &rcs::sim::SimGripperConfig::max_joint_width)
      .def_readwrite("min_joint_width",
                     &rcs::sim::SimGripperConfig::min_joint_width)
      .def_readwrite("actuator", &rcs::sim::SimGripperConfig::actuator)
      .def_readwrite("max_actuator_width",
                     &rcs::sim::SimGripperConfig::max_actuator_width)
      .def_readwrite("min_actuator_width",
                     &rcs::sim::SimGripperConfig::min_actuator_width)
      .def("add_id", &rcs::sim::SimGripperConfig::add_id, py::arg("id"));
  py::class_<rcs::sim::SimGripperState, rcs::common::GripperState>(
      sim, "SimGripperState")
      .def(py::init<>())
      .def_readonly("last_commanded_width",
                    &rcs::sim::SimGripperState::last_commanded_width)
      .def_readonly("is_moving", &rcs::sim::SimGripperState::is_moving)
      .def_readonly("last_width", &rcs::sim::SimGripperState::last_width)
      .def_readonly("collision", &rcs::sim::SimGripperState::collision);

  py::class_<rcs::sim::Sim, std::shared_ptr<rcs::sim::Sim>>(sim, "Sim")
      .def(py::init([](long m, long d) {
             return std::make_shared<rcs::sim::Sim>((mjModel *)m, (mjData *)d);
           }),
           py::arg("mjmdl"), py::arg("mjdata"))
      .def("step_until_convergence", &rcs::sim::Sim::step_until_convergence,
           py::call_guard<py::gil_scoped_release>())
      .def("is_converged", &rcs::sim::Sim::is_converged)
      .def("step", &rcs::sim::Sim::step, py::arg("k"))
      .def("reset", &rcs::sim::Sim::reset)
      .def("_start_gui_server", &rcs::sim::Sim::start_gui_server, py::arg("id"))
      .def("_stop_gui_server", &rcs::sim::Sim::stop_gui_server);
  py::class_<rcs::sim::SimGripper, rcs::common::Gripper,
             std::shared_ptr<rcs::sim::SimGripper>>(sim, "SimGripper")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    const rcs::sim::SimGripperConfig &>(),
           py::arg("sim"), py::arg("cfg"))
      .def("get_parameters", &rcs::sim::SimGripper::get_parameters)
      .def("get_state", &rcs::sim::SimGripper::get_state)
      .def("set_parameters", &rcs::sim::SimGripper::set_parameters,
           py::arg("cfg"));
  py::class_<rcs::sim::SimRobot, rcs::common::Robot,
             std::shared_ptr<rcs::sim::SimRobot>>(sim, "SimRobot")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    std::shared_ptr<rcs::common::IK>, rcs::sim::SimRobotConfig,
                    bool>(),
           py::arg("sim"), py::arg("ik"), py::arg("cfg"),
           py::arg("register_convergence_callback") = true)
      .def("get_parameters", &rcs::sim::SimRobot::get_parameters)
      .def("set_parameters", &rcs::sim::SimRobot::set_parameters,
           py::arg("cfg"))
      .def("set_joints_hard", &rcs::sim::SimRobot::set_joints_hard,
           py::arg("q"))
      .def("get_state", &rcs::sim::SimRobot::get_state);
  
  // SimTilburgHandState
  py::class_<rcs::sim::SimTilburgHandState, rcs::common::HandState>(
      sim, "SimTilburgHandState")
      .def(py::init<>())
      .def_readonly("last_commanded_qpos",
                    &rcs::sim::SimTilburgHandState::last_commanded_qpos)
      .def_readonly("is_moving", &rcs::sim::SimTilburgHandState::is_moving)
      .def_readonly("last_qpos", &rcs::sim::SimTilburgHandState::last_qpos)
      .def_readonly("collision", &rcs::sim::SimTilburgHandState::collision);
  // SimTilburgHandConfig
  py::class_<rcs::sim::SimTilburgHandConfig, rcs::common::HandConfig>(
      sim, "SimTilburgHandConfig")
      .def(py::init<>())
      .def_readwrite("max_joint_position",
                     &rcs::sim::SimTilburgHandConfig::max_joint_position)
      .def_readwrite("min_joint_position",
                     &rcs::sim::SimTilburgHandConfig::min_joint_position)
      .def_readwrite("ignored_collision_geoms",
                     &rcs::sim::SimTilburgHandConfig::ignored_collision_geoms)
      .def_readwrite("collision_geoms",
                     &rcs::sim::SimTilburgHandConfig::collision_geoms)
      .def_readwrite("collision_geoms_fingers",
                     &rcs::sim::SimTilburgHandConfig::collision_geoms_fingers)
      .def_readwrite("joints", &rcs::sim::SimTilburgHandConfig::joints)
      .def_readwrite("actuators", &rcs::sim::SimTilburgHandConfig::actuators)
      .def_readwrite("grasp_type",
                     &rcs::sim::SimTilburgHandConfig::grasp_type)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::SimTilburgHandConfig::seconds_between_callbacks)
      .def("add_id", &rcs::sim::SimTilburgHandConfig::add_id, py::arg("id"));
  // SimTilburgHand
  py::class_<rcs::sim::SimTilburgHand, rcs::common::Hand,
             std::shared_ptr<rcs::sim::SimTilburgHand>>(sim, "SimTilburgHand")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    const rcs::sim::SimTilburgHandConfig &>(),
           py::arg("sim"), py::arg("cfg"))
      .def("get_parameters", &rcs::sim::SimTilburgHand::get_parameters)
      .def("get_state", &rcs::sim::SimTilburgHand::get_state)
      .def("set_parameters", &rcs::sim::SimTilburgHand::set_parameters,
           py::arg("cfg"));

  py::enum_<rcs::sim::CameraType>(sim, "CameraType")
      .value("free", rcs::sim::CameraType::free)
      .value("tracking", rcs::sim::CameraType::tracking)
      .value("fixed", rcs::sim::CameraType::fixed)
      .value("default_free", rcs::sim::CameraType::default_free)
      .export_values();
  py::class_<rcs::sim::SimCameraConfig, rcs::common::BaseCameraConfig>(
      sim, "SimCameraConfig")
      .def(py::init<const std::string &, int, int, int, rcs::sim::CameraType>(),
           py::arg("identifier"), py::arg("frame_rate"),
           py::arg("resolution_width"), py::arg("resolution_height"),
           py::arg("type") = rcs::sim::CameraType::fixed)
      .def_readwrite("type", &rcs::sim::SimCameraConfig::type);
  py::class_<rcs::sim::FrameSet>(sim, "FrameSet")
      .def(py::init<>())
      .def_readonly("color_frames", &rcs::sim::FrameSet::color_frames)
      .def_readonly("depth_frames", &rcs::sim::FrameSet::depth_frames)
      .def_readonly("timestamp", &rcs::sim::FrameSet::timestamp);
  py::class_<rcs::sim::SimCameraSet>(sim, "SimCameraSet")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    std::unordered_map<std::string, rcs::sim::SimCameraConfig>,
                    bool>(),
           py::arg("sim"), py::arg("cameras"),
           py::arg("render_on_demand") = true)
      .def("buffer_size", &rcs::sim::SimCameraSet::buffer_size)
      .def("clear_buffer", &rcs::sim::SimCameraSet::clear_buffer)
      .def("get_latest_frameset", &rcs::sim::SimCameraSet::get_latest_frameset)
      .def_property_readonly("_sim", &rcs::sim::SimCameraSet::get_sim)
      .def("get_timestamp_frameset",
           &rcs::sim::SimCameraSet::get_timestamp_frameset, py::arg("ts"));
  py::class_<rcs::sim::GuiClient>(sim, "GuiClient")
      .def(py::init<const std::string &>(), py::arg("id"))
      .def("get_model_bytes",
           [](const rcs::sim::GuiClient &self) {
             auto s = self.get_model_bytes();
             return py::bytes(s);
           })
      .def("set_model_and_data",
           [](rcs::sim::GuiClient &self, long m, long d) {
             self.set_model_and_data((mjModel *)m, (mjData *)d);
           })
      .def("sync", &rcs::sim::GuiClient::sync);
}
