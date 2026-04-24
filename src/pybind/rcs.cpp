#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sim/SimGripper.h>
#include <sim/SimRobot.h>
#include <sim/SimTilburgHand.h>
#include <sim/camera.h>
#include <sim/gui.h>

#include <functional>
#include <memory>

#include "rcs/Kinematics.h"
#include "rcs/Pose.h"
#include "rcs/Robot.h"
#include "rcs/utils.h"

// TODO: define exceptions

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

template <typename T>
py::class_<T> bind_type_class(py::module_& m, const std::string& class_name) {
  return py::class_<T>(m, class_name.c_str())
      .def(py::init<std::string>())
      .def_static("get_all", &T::get_all)
      .def_readonly("id", &T::id)
      // just uses the id, this allows us to use strings also
      // for comparison and dictionary lookups
      .def("__hash__", [](const T& t) { return py::hash(py::str(t.id)); })
      .def("__eq__",
           [](const T& self, py::handle other) {
             if (py::isinstance<T>(other)) {
               return self.id == py::cast<T>(other).id;
             }
             if (py::isinstance<py::str>(other)) {
               return self.id == py::cast<std::string>(other);
             }
             return false;
           })
      .def("__repr__", [class_name](const T& t) {
        return "<" + class_name + ": " + t.id + ">";
      });
}

/**
 * @brief Robot trampoline class for python bindings,
 * needed for pybind11 to override virtual functions,
 * see
 * https://pybind11.readthedocs.io/en/stable/advanced/classes.html
 */
class PyRobot : public rcs::common::Robot {
 public:
  using rcs::common::Robot::Robot;  // Inherit constructors

  rcs::common::RobotConfig* get_config() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RobotConfig*, rcs::common::Robot,
                           get_config, );
  }

  rcs::common::RobotState* get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::RobotState*, rcs::common::Robot,
                           get_state, );
  }

  rcs::common::Pose get_cartesian_position() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           get_cartesian_position, );
  }

  void set_joint_position(const rcs::common::VectorXd& q) override {
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

  void close() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Robot, close, );
  }
  std::optional<std::shared_ptr<rcs::common::Kinematics>> get_ik() override {
    PYBIND11_OVERRIDE_PURE(
        std::optional<std::shared_ptr<rcs::common::Kinematics>>,
        rcs::common::Robot, get_ik, );
  }
  rcs::common::Pose get_base_pose_in_world_coordinates() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Robot,
                           get_base_pose_in_world_coordinates, );
  }

  void set_cartesian_position(const rcs::common::Pose& pose) override {
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

  rcs::common::GripperConfig* get_config() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GripperConfig*, rcs::common::Gripper,
                           get_config, );
  }

  rcs::common::GripperState* get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::GripperState*, rcs::common::Gripper,
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
  void close() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Gripper, close, );
  }
};

/**
 * @brief Hand trampoline class for python bindings
 */
class PyHand : public rcs::common::Hand {
 public:
  using rcs::common::Hand::Hand;  // Inherit constructors

  rcs::common::HandConfig* get_config() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::HandConfig*, rcs::common::Hand,
                           get_config, );
  }

  rcs::common::HandState* get_state() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::HandState*, rcs::common::Hand,
                           get_state, );
  }

  void set_normalized_joint_poses(const rcs::common::VectorXd& q) override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, set_normalized_joint_poses,
                           q);
  }

  rcs::common::VectorXd get_normalized_joint_poses() override {
    PYBIND11_OVERRIDE_PURE(rcs::common::VectorXd, rcs::common::Hand,
                           get_normalized_joint_poses, );
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
  void close() override {
    PYBIND11_OVERRIDE_PURE(void, rcs::common::Hand, close, );
  }
};

// trampoline kinematics
class PyKinematics : public rcs::common::Kinematics {
  // TODO: use line below when upgraded to pybind11 3.x
  //  public py::trampoline_self_life_support {
 public:
  using rcs::common::Kinematics::Kinematics;  // Inherit constructors

  std::optional<rcs::common::VectorXd> inverse(
      const rcs::common::Pose& pose, const rcs::common::VectorXd& q0,
      const rcs::common::Pose& tcp_offset =
          rcs::common::Pose::Identity()) override {
    PYBIND11_OVERRIDE_PURE(std::optional<rcs::common::VectorXd>,
                           rcs::common::Kinematics, inverse, pose, q0,
                           tcp_offset);
  }

  rcs::common::Pose forward(const rcs::common::VectorXd& q0,
                            const rcs::common::Pose& tcp_offset) override {
    PYBIND11_OVERRIDE_PURE(rcs::common::Pose, rcs::common::Kinematics, forward,
                           q0, tcp_offset);
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
      .def(py::init([](const std::string& identifier, int frame_rate,
                       int resolution_width, int resolution_height) {
             auto cfg = new rcs::common::BaseCameraConfig();
             cfg->identifier = identifier;
             cfg->frame_rate = frame_rate;
             cfg->resolution_width = resolution_width;
             cfg->resolution_height = resolution_height;
             return cfg;
           }),
           py::arg("identifier"), py::arg("frame_rate"),
           py::arg("resolution_width"), py::arg("resolution_height"))
      .def_readwrite("identifier", &rcs::common::BaseCameraConfig::identifier)
      .def_readwrite("frame_rate", &rcs::common::BaseCameraConfig::frame_rate)
      .def_readwrite("resolution_width",
                     &rcs::common::BaseCameraConfig::resolution_width)
      .def_readwrite("resolution_height",
                     &rcs::common::BaseCameraConfig::resolution_height)
      .def("__copy__",
           [](const rcs::common::BaseCameraConfig& self) {
             return rcs::common::BaseCameraConfig(self);
           })
      .def("__deepcopy__",
           [](const rcs::common::BaseCameraConfig& self, py::dict) {
             return rcs::common::BaseCameraConfig(self);
           });
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
          [](const rcs::common::RPY& p) {  // dump
            return py::make_tuple(p.roll, p.pitch, p.yaw);
          },
          [](py::tuple t) {  // load
            return rcs::common::RPY(t[0].cast<double>(), t[1].cast<double>(),
                                    t[2].cast<double>());
          }));

  py::class_<rcs::common::RotVec>(common, "RotVec")
      .def(py::init<Eigen::Vector3d>(), py::arg("vec"))
      .def("rotation_matrix", &rcs::common::RotVec::rotation_matrix)
      .def("as_vector", &rcs::common::RotVec::as_vector)
      .def("as_quaternion_vector", &rcs::common::RotVec::as_quaternion_vector)
      .def("is_close", &rcs::common::RotVec::is_close, py::arg("other"),
           py::arg("eps") = 1e-8)
      .def("__str__", &rcs::common::RotVec::str);

  py::class_<rcs::common::Pose>(common, "Pose")
      .def(py::init<>())
      .def(py::init<const Eigen::Matrix4d&>(), py::arg("pose_matrix"))
      .def(py::init<const Eigen::Matrix3d&, const Eigen::Vector3d&>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&>(),
           py::arg("quaternion"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY&, const Eigen::Vector3d&>(),
           py::arg("rpy"), py::arg("translation"))
      .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
           py::arg("rpy_vector"), py::arg("translation"))
      .def(py::init<const Eigen::Vector3d&>(), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d&>(), py::arg("quaternion"))
      .def(py::init<const rcs::common::RPY&>(), py::arg("rpy"))
      .def(py::init<const Eigen::Matrix3d&>(), py::arg("rotation"))
      .def(py::init<const rcs::common::Pose&>(), py::arg("pose"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("rotation_q_wxyz", &rcs::common::Pose::rotation_q_wxyz)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rotation_rpy", &rcs::common::Pose::rotation_rpy)
      .def("xyzrpy", &rcs::common::Pose::xyzrpy)
      .def("rotvec", &rcs::common::Pose::rotvec)
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
          [](const rcs::common::Pose& p) {  // dump
            return p.affine_array();
          },
          [](std::array<double, 16> t) {  // load
            return rcs::common::Pose(t);
          }));

  py::class_<rcs::common::Kinematics, PyKinematics,
             std::shared_ptr<rcs::common::Kinematics>>(
      // TODO: use line below when upgraded to pybind11 3.x
      // py::class_<rcs::common::Kinematics, PyKinematics, py::smart_holder>(
      common, "Kinematics")
      .def(py::init<>())
      .def("inverse", &rcs::common::Kinematics::inverse, py::arg("pose"),
           py::arg("q0"), py::arg("tcp_offset") = rcs::common::Pose::Identity())
      .def("forward", &rcs::common::Kinematics::forward, py::arg("q0"),
           py::arg("tcp_offset") = rcs::common::Pose::Identity());

  py::class_<rcs::common::Pin, rcs::common::Kinematics,
             std::shared_ptr<rcs::common::Pin>>(common, "Pin")
      .def(py::init<const std::string&, const std::string&, bool>(),
           py::arg("path"), py::arg("frame_id") = "fr3_link8",
           py::arg("urdf") = false);

  bind_type_class<rcs::common::RobotType>(common, "RobotType")
      .def_readonly_static("FR3", &rcs::common::RobotType::FR3)
      .def_readonly_static("Panda", &rcs::common::RobotType::Panda);

  py::enum_<rcs::common::RobotPlatform>(common, "RobotPlatform")
      .value("HARDWARE", rcs::common::RobotPlatform::HARDWARE)
      .value("SIMULATION", rcs::common::RobotPlatform::SIMULATION)
      .export_values();

  rcs::common::RobotConfig default_robot_config;
  py::class_<rcs::common::RobotConfig>(common, "RobotConfig")
      .def(py::init([](rcs::common::RobotType robot_type, size_t dof,
                       const Eigen::Matrix<double, 2, Eigen::Dynamic,
                                           Eigen::ColMajor>& joint_limits,
                       rcs::common::RobotPlatform robot_platform,
                       const rcs::common::Pose& tcp_offset,
                       const std::string& attachment_site,
                       const std::string& kinematic_model_path,
                       std::optional<rcs::common::VectorXd> q_home) {
             rcs::common::RobotConfig config;
             config.robot_type = robot_type;
             config.robot_platform = robot_platform;
             config.tcp_offset = tcp_offset;
             config.attachment_site = attachment_site;
             config.kinematic_model_path = kinematic_model_path;
             config.q_home = q_home;
             config.dof = dof;
             config.joint_limits = joint_limits;
             return config;
           }),
           py::arg("robot_type") = default_robot_config.robot_type,
           py::arg("dof") = default_robot_config.dof,
           py::arg("joint_limits") = default_robot_config.joint_limits,
           py::arg("robot_platform") = default_robot_config.robot_platform,
           py::arg("tcp_offset") = default_robot_config.tcp_offset,
           py::arg("attachment_site") = default_robot_config.attachment_site,
           py::arg("kinematic_model_path") =
               default_robot_config.kinematic_model_path,
           py::arg("q_home") = default_robot_config.q_home)
      .def_readwrite("robot_type", &rcs::common::RobotConfig::robot_type)
      .def_readwrite("dof", &rcs::common::RobotConfig::dof)
      .def_readwrite("joint_limits", &rcs::common::RobotConfig::joint_limits)
      .def_readwrite("kinematic_model_path",
                     &rcs::common::RobotConfig::kinematic_model_path)
      .def_readwrite("attachment_site",
                     &rcs::common::RobotConfig::attachment_site)
      .def_readwrite("tcp_offset", &rcs::common::RobotConfig::tcp_offset)
      .def_readwrite("robot_platform",
                     &rcs::common::RobotConfig::robot_platform)
      .def_readwrite("q_home", &rcs::common::RobotConfig::q_home);
  py::class_<rcs::common::RobotState>(common, "RobotState").def(py::init<>());

  bind_type_class<rcs::common::GripperType>(common, "GripperType")
      .def_readonly_static("FrankaHand", &rcs::common::GripperType::FrankaHand);

  rcs::common::GripperConfig default_gripper_config;
  py::class_<rcs::common::GripperConfig>(common, "GripperConfig")
      .def(py::init([](rcs::common::GripperType gripper_type) {
             rcs::common::GripperConfig config;
             config.gripper_type = gripper_type;
             return config;
           }),
           py::arg("gripper_type") = default_gripper_config.gripper_type)
      .def_readwrite("gripper_type", &rcs::common::GripperConfig::gripper_type);
  py::class_<rcs::common::GripperState>(common, "GripperState")
      .def(py::init<>());
  py::enum_<rcs::common::GraspType>(common, "GraspType")
      .value("POWER_GRASP", rcs::common::GraspType::POWER_GRASP)
      .value("PRECISION_GRASP", rcs::common::GraspType::PRECISION_GRASP)
      .value("LATERAL_GRASP", rcs::common::GraspType::LATERAL_GRASP)
      .value("TRIPOD_GRASP", rcs::common::GraspType::TRIPOD_GRASP)
      .export_values();
  py::class_<rcs::common::HandConfig>(common, "HandConfig").def(py::init<>());
  py::class_<rcs::common::HandState>(common, "HandState").def(py::init<>());

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, PyRobot, std::shared_ptr<rcs::common::Robot>>(
      common, "Robot")
      .def(py::init<>())
      .def("get_config", &rcs::common::Robot::get_config)
      .def("get_state", &rcs::common::Robot::get_state)
      .def("get_cartesian_position",
           &rcs::common::Robot::get_cartesian_position)
      .def("set_joint_position", &rcs::common::Robot::set_joint_position,
           py::arg("q"), py::call_guard<py::gil_scoped_release>())
      .def("get_joint_position", &rcs::common::Robot::get_joint_position)
      .def("move_home", &rcs::common::Robot::move_home,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Robot::reset)
      .def("close", &rcs::common::Robot::close)
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
      .def(py::init<>())
      .def("get_config", &rcs::common::Gripper::get_config)
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
      .def("close", &rcs::common::Gripper::close,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Gripper::reset,
           py::call_guard<py::gil_scoped_release>());

  py::class_<rcs::common::Hand, PyHand, std::shared_ptr<rcs::common::Hand>>(
      common, "Hand")
      .def(py::init<>())
      .def("get_config", &rcs::common::Hand::get_config)
      .def("get_state", &rcs::common::Hand::get_state)
      .def("set_normalized_joint_poses",
           &rcs::common::Hand::set_normalized_joint_poses, py::arg("q"))
      .def("get_normalized_joint_poses",
           &rcs::common::Hand::get_normalized_joint_poses)
      .def("grasp", &rcs::common::Hand::grasp,
           py::call_guard<py::gil_scoped_release>())
      .def("is_grasped", &rcs::common::Hand::is_grasped)
      .def("open", &rcs::common::Hand::open,
           py::call_guard<py::gil_scoped_release>())
      .def("shut", &rcs::common::Hand::shut,
           py::call_guard<py::gil_scoped_release>())
      .def("close", &rcs::common::Hand::close,
           py::call_guard<py::gil_scoped_release>())
      .def("reset", &rcs::common::Hand::reset,
           py::call_guard<py::gil_scoped_release>());

  // SIM MODULE
  auto sim = m.def_submodule("sim", "sim module");
  rcs::sim::SimRobotConfig default_simrobot_cfg = rcs::sim::SimRobotConfig();
  py::class_<rcs::sim::SimRobotConfig, rcs::common::RobotConfig>(
      sim, "SimRobotConfig")
      .def(
          py::init([](rcs::common::RobotType robot_type,
                      rcs::common::Pose tcp_offset, std::string attachment_site,
                      std::string kinematic_model_path,
                      double joint_rotational_tolerance,
                      double seconds_between_callbacks, bool trajectory_trace,
                      std::vector<std::string> arm_collision_geoms,
                      std::vector<std::string> joints,
                      std::optional<rcs::common::VectorXd> q_home,
                      std::vector<std::string> actuators, std::string base,
                      size_t dof,
                      const Eigen::Matrix<double, 2, Eigen::Dynamic,
                                          Eigen::ColMajor>& joint_limits) {
            rcs::sim::SimRobotConfig config;
            config.robot_type = robot_type;
            config.robot_platform = rcs::common::RobotPlatform::SIMULATION;
            config.tcp_offset = tcp_offset;
            config.attachment_site = attachment_site;
            config.kinematic_model_path = kinematic_model_path;
            config.joint_rotational_tolerance = joint_rotational_tolerance;
            config.seconds_between_callbacks = seconds_between_callbacks;
            config.trajectory_trace = trajectory_trace;
            config.arm_collision_geoms = arm_collision_geoms;
            config.joints = joints;
            config.actuators = actuators;
            config.base = base;
            config.dof = dof;
            config.joint_limits = joint_limits;
            config.q_home = q_home;
            return config;
          }),
          py::arg("robot_type") = default_simrobot_cfg.robot_type,
          py::arg("tcp_offset") = default_simrobot_cfg.tcp_offset,
          py::arg("attachment_site") = default_simrobot_cfg.attachment_site,
          py::arg("kinematic_model_path") =
              default_simrobot_cfg.kinematic_model_path,
          py::arg("joint_rotational_tolerance") =
              default_simrobot_cfg.joint_rotational_tolerance,
          py::arg("seconds_between_callbacks") =
              default_simrobot_cfg.seconds_between_callbacks,
          py::arg("trajectory_trace") = default_simrobot_cfg.trajectory_trace,
          py::arg("arm_collision_geoms") =
              default_simrobot_cfg.arm_collision_geoms,
          py::arg("joints") = default_simrobot_cfg.joints,
          py::arg("q_home") = default_simrobot_cfg.q_home,
          py::arg("actuators") = default_simrobot_cfg.actuators,
          py::arg("base") = default_simrobot_cfg.base,
          py::arg("dof") = default_simrobot_cfg.dof,
          py::arg("joint_limits") = default_simrobot_cfg.joint_limits)

      .def_readwrite("joint_rotational_tolerance",
                     &rcs::sim::SimRobotConfig::joint_rotational_tolerance)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::SimRobotConfig::seconds_between_callbacks)
      .def_readwrite("trajectory_trace",
                     &rcs::sim::SimRobotConfig::trajectory_trace)
      .def_readwrite("arm_collision_geoms",
                     &rcs::sim::SimRobotConfig::arm_collision_geoms)
      .def_readwrite("joints", &rcs::sim::SimRobotConfig::joints)
      .def_readwrite("actuators", &rcs::sim::SimRobotConfig::actuators)
      .def_readwrite("base", &rcs::sim::SimRobotConfig::base)
      .def_readwrite("dof", &rcs::sim::SimRobotConfig::dof)
      .def_readwrite("joint_limits", &rcs::sim::SimRobotConfig::joint_limits)
      .def("__copy__",
           [](const rcs::sim::SimRobotConfig& self) {
             return rcs::sim::SimRobotConfig(self);
           })
      .def("__deepcopy__",
           [](const rcs::sim::SimRobotConfig& self, py::dict) {
             return rcs::sim::SimRobotConfig(self);
           })
      .def("add_prefix", &rcs::sim::SimRobotConfig::add_prefix, py::arg("id"));
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

  rcs::sim::SimGripperConfig default_simgripper_cfg =
      rcs::sim::SimGripperConfig();
  py::class_<rcs::sim::SimGripperConfig, rcs::common::GripperConfig>(
      sim, "SimGripperConfig")
      .def(py::init([](double epsilon_inner, double epsilon_outer,
                       double seconds_between_callbacks,
                       std::vector<std::string> ignored_collision_geoms,
                       std::vector<std::string> collision_geoms,
                       std::vector<std::string> collision_geoms_fingers,
                       std::vector<std::string> joints, double max_joint_width,
                       double min_joint_width, std::string actuator,
                       double max_actuator_width, double min_actuator_width,
                       rcs::common::GripperType gripper_type) {
             rcs::sim::SimGripperConfig config;
             config.epsilon_inner = epsilon_inner;
             config.epsilon_outer = epsilon_outer;
             config.seconds_between_callbacks = seconds_between_callbacks;
             config.ignored_collision_geoms = ignored_collision_geoms;
             config.collision_geoms = collision_geoms;
             config.collision_geoms_fingers = collision_geoms_fingers;
             config.joints = joints;
             config.max_joint_width = max_joint_width;
             config.min_joint_width = min_joint_width;
             config.actuator = actuator;
             config.max_actuator_width = max_actuator_width;
             config.min_actuator_width = min_actuator_width;
             config.gripper_type = gripper_type;
             return config;
           }),
           py::arg("epsilon_inner") = default_simgripper_cfg.epsilon_inner,
           py::arg("epsilon_outer") = default_simgripper_cfg.epsilon_outer,
           py::arg("seconds_between_callbacks") =
               default_simgripper_cfg.seconds_between_callbacks,
           py::arg("ignored_collision_geoms") =
               default_simgripper_cfg.ignored_collision_geoms,
           py::arg("collision_geoms") = default_simgripper_cfg.collision_geoms,
           py::arg("collision_geoms_fingers") =
               default_simgripper_cfg.collision_geoms_fingers,
           py::arg("joints") = default_simgripper_cfg.joints,
           py::arg("max_joint_width") = default_simgripper_cfg.max_joint_width,
           py::arg("min_joint_width") = default_simgripper_cfg.min_joint_width,
           py::arg("actuator") = default_simgripper_cfg.actuator,
           py::arg("max_actuator_width") =
               default_simgripper_cfg.max_actuator_width,
           py::arg("min_actuator_width") =
               default_simgripper_cfg.min_actuator_width,
           py::arg("gripper_type") = default_simgripper_cfg.gripper_type)
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
      .def_readwrite("joints", &rcs::sim::SimGripperConfig::joints)
      .def_readwrite("max_joint_width",
                     &rcs::sim::SimGripperConfig::max_joint_width)
      .def_readwrite("min_joint_width",
                     &rcs::sim::SimGripperConfig::min_joint_width)
      .def_readwrite("actuator", &rcs::sim::SimGripperConfig::actuator)
      .def_readwrite("max_actuator_width",
                     &rcs::sim::SimGripperConfig::max_actuator_width)
      .def_readwrite("min_actuator_width",
                     &rcs::sim::SimGripperConfig::min_actuator_width)
      .def_readwrite("gripper_type", &rcs::sim::SimGripperConfig::gripper_type)
      .def("__copy__",
           [](const rcs::sim::SimGripperConfig& self) {
             return rcs::sim::SimGripperConfig(self);
           })
      .def("__deepcopy__",
           [](const rcs::sim::SimGripperConfig& self, py::dict) {
             return rcs::sim::SimGripperConfig(self);
           })
      .def("add_prefix", &rcs::sim::SimGripperConfig::add_prefix,
           py::arg("id"));
  py::class_<rcs::sim::SimGripperState, rcs::common::GripperState>(
      sim, "SimGripperState")
      .def(py::init<>())
      .def_readonly("last_commanded_width",
                    &rcs::sim::SimGripperState::last_commanded_width)
      .def_readonly("is_moving", &rcs::sim::SimGripperState::is_moving)
      .def_readonly("last_width", &rcs::sim::SimGripperState::last_width)
      .def_readonly("collision", &rcs::sim::SimGripperState::collision);

  rcs::sim::SimConfig default_sim_cfg = rcs::sim::SimConfig();
  py::class_<rcs::sim::SimConfig>(sim, "SimConfig")
      .def(py::init([](bool async_control, bool realtime, double frequency,
                       int max_convergence_steps) {
             rcs::sim::SimConfig config;
             config.async_control = async_control;
             config.realtime = realtime;
             config.frequency = frequency;
             config.max_convergence_steps = max_convergence_steps;
             return config;
           }),
           py::arg("async_control") = default_sim_cfg.async_control,
           py::arg("realtime") = default_sim_cfg.realtime,
           py::arg("frequency") = default_sim_cfg.frequency,
           py::arg("max_convergence_steps") =
               default_sim_cfg.max_convergence_steps)
      .def_readwrite("async_control", &rcs::sim::SimConfig::async_control)
      .def_readwrite("realtime", &rcs::sim::SimConfig::realtime)
      .def_readwrite("frequency", &rcs::sim::SimConfig::frequency)
      .def_readwrite("max_convergence_steps",
                     &rcs::sim::SimConfig::max_convergence_steps)
      .def("__copy__",
           [](const rcs::sim::SimConfig& self) {
             return rcs::sim::SimConfig(self);
           })
      .def("__deepcopy__", [](const rcs::sim::SimConfig& self, py::dict) {
        return rcs::sim::SimConfig(self);
      });

  py::class_<rcs::sim::Sim, std::shared_ptr<rcs::sim::Sim>>(sim, "Sim")
      .def(py::init([](long m, long d) {
             return std::make_shared<rcs::sim::Sim>((mjModel*)m, (mjData*)d);
           }),
           py::arg("mjmdl"), py::arg("mjdata"))
      .def("step_until_convergence", &rcs::sim::Sim::step_until_convergence,
           py::call_guard<py::gil_scoped_release>())
      .def("is_converged", &rcs::sim::Sim::is_converged)
      .def("set_config", &rcs::sim::Sim::set_config, py::arg("cfg"))
      .def("get_config", &rcs::sim::Sim::get_config)
      .def("step", &rcs::sim::Sim::step, py::arg("k"))
      .def("reset", &rcs::sim::Sim::reset)
      .def("_start_gui_server", &rcs::sim::Sim::start_gui_server, py::arg("id"))
      .def("_stop_gui_server", &rcs::sim::Sim::stop_gui_server);

  py::class_<rcs::sim::SimGripper, rcs::common::Gripper,
             std::shared_ptr<rcs::sim::SimGripper>>(sim, "SimGripper")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    const rcs::sim::SimGripperConfig&>(),
           py::arg("sim"), py::arg("cfg"))
      .def("get_config", &rcs::sim::SimGripper::get_config)
      .def("get_state", &rcs::sim::SimGripper::get_state)
      .def("clear_collision_flag", &rcs::sim::SimGripper::clear_collision_flag)
      .def("set_config", &rcs::sim::SimGripper::set_config, py::arg("cfg"));
  py::class_<rcs::sim::SimRobot, rcs::common::Robot,
             std::shared_ptr<rcs::sim::SimRobot>>(sim, "SimRobot")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    std::shared_ptr<rcs::common::Kinematics>,
                    rcs::sim::SimRobotConfig, bool>(),
           py::arg("sim"), py::arg("ik"), py::arg("cfg"),
           py::arg("register_convergence_callback") = true)
      .def("get_config", &rcs::sim::SimRobot::get_config)
      .def("set_config", &rcs::sim::SimRobot::set_config, py::arg("cfg"))
      .def("set_joints_hard", &rcs::sim::SimRobot::set_joints_hard,
           py::arg("q"))
      .def("clear_collision_flag", &rcs::sim::SimRobot::clear_collision_flag)
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
  rcs::sim::SimTilburgHandConfig default_simtilburghand_cfg =
      rcs::sim::SimTilburgHandConfig();
  py::class_<rcs::sim::SimTilburgHandConfig, rcs::common::HandConfig>(
      sim, "SimTilburgHandConfig")
      .def(py::init([](rcs::common::GraspType grasp_type,
                       double seconds_between_callbacks,
                       std::vector<std::string> ignored_collision_geoms,
                       std::vector<std::string> collision_geoms,
                       std::vector<std::string> collision_geoms_fingers,
                       std::vector<std::string> joints,
                       std::vector<std::string> actuators,
                       rcs::common::Vector16d max_joint_position,
                       rcs::common::Vector16d min_joint_position) {
             rcs::sim::SimTilburgHandConfig config;
             config.grasp_type = grasp_type;
             config.seconds_between_callbacks = seconds_between_callbacks;
             config.ignored_collision_geoms = ignored_collision_geoms;
             config.collision_geoms = collision_geoms;
             config.collision_geoms_fingers = collision_geoms_fingers;
             config.joints = joints;
             config.actuators = actuators;
             config.max_joint_position = max_joint_position;
             config.min_joint_position = min_joint_position;
             return config;
           }),
           py::arg("grasp_type") = default_simtilburghand_cfg.grasp_type,
           py::arg("seconds_between_callbacks") =
               default_simtilburghand_cfg.seconds_between_callbacks,
           py::arg("ignored_collision_geoms") =
               default_simtilburghand_cfg.ignored_collision_geoms,
           py::arg("collision_geoms") =
               default_simtilburghand_cfg.collision_geoms,
           py::arg("collision_geoms_fingers") =
               default_simtilburghand_cfg.collision_geoms_fingers,
           py::arg("joints") = default_simtilburghand_cfg.joints,
           py::arg("actuators") = default_simtilburghand_cfg.actuators,
           py::arg("max_joint_position") =
               default_simtilburghand_cfg.max_joint_position,
           py::arg("min_joint_position") =
               default_simtilburghand_cfg.min_joint_position)
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
      .def_readwrite("grasp_type", &rcs::sim::SimTilburgHandConfig::grasp_type)
      .def_readwrite("seconds_between_callbacks",
                     &rcs::sim::SimTilburgHandConfig::seconds_between_callbacks)
      .def("add_prefix", &rcs::sim::SimTilburgHandConfig::add_prefix,
           py::arg("id"))
      .def("__copy__",
           [](const rcs::sim::SimTilburgHandConfig& self) {
             return rcs::sim::SimTilburgHandConfig(self);
           })
      .def("__deepcopy__",
           [](const rcs::sim::SimTilburgHandConfig& self, py::dict) {
             return rcs::sim::SimTilburgHandConfig(self);
           });
  // SimTilburgHand
  py::class_<rcs::sim::SimTilburgHand, rcs::common::Hand,
             std::shared_ptr<rcs::sim::SimTilburgHand>>(sim, "SimTilburgHand")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    const rcs::sim::SimTilburgHandConfig&>(),
           py::arg("sim"), py::arg("cfg"))
      .def("get_config", &rcs::sim::SimTilburgHand::get_config)
      .def("get_state", &rcs::sim::SimTilburgHand::get_state)
      .def("set_config", &rcs::sim::SimTilburgHand::set_config, py::arg("cfg"));

  py::enum_<rcs::sim::CameraType>(sim, "CameraType")
      .value("free", rcs::sim::CameraType::free)
      .value("tracking", rcs::sim::CameraType::tracking)
      .value("fixed", rcs::sim::CameraType::fixed)
      .value("default_free", rcs::sim::CameraType::default_free)
      .export_values();
  py::class_<rcs::sim::SimCameraConfig, rcs::common::BaseCameraConfig>(
      sim, "SimCameraConfig")
      .def(py::init([](const std::string& identifier, int frame_rate,
                       int resolution_width, int resolution_height,
                       rcs::sim::CameraType type) {
             rcs::sim::SimCameraConfig config;
             config.identifier = identifier;
             config.frame_rate = frame_rate;
             config.resolution_width = resolution_width;
             config.resolution_height = resolution_height;
             config.type = type;
             return config;
           }),
           py::arg("identifier"), py::arg("frame_rate"),
           py::arg("resolution_width"), py::arg("resolution_height"),
           py::arg("type") = rcs::sim::CameraType::fixed)
      .def_readwrite("type", &rcs::sim::SimCameraConfig::type)
      .def("__copy__",
           [](const rcs::sim::SimCameraConfig& self) {
             return rcs::sim::SimCameraConfig(self);
           })
      .def("__deepcopy__", [](const rcs::sim::SimCameraConfig& self, py::dict) {
        return rcs::sim::SimCameraConfig(self);
      });
  py::class_<rcs::sim::FrameSet>(sim, "FrameSet")
      .def(py::init(
               [](const std::unordered_map<std::string, rcs::sim::ColorFrame>&
                      color_frames,
                  const std::unordered_map<std::string, rcs::sim::DepthFrame>&
                      depth_frames,
                  double timestamp) {
                 rcs::sim::FrameSet fs;
                 fs.color_frames = color_frames;
                 fs.depth_frames = depth_frames;
                 fs.timestamp = timestamp;
                 return fs;
               }),
           py::arg("color_frames"), py::arg("depth_frames"),
           py::arg("timestamp"))
      .def_readonly("color_frames", &rcs::sim::FrameSet::color_frames)
      .def_readonly("depth_frames", &rcs::sim::FrameSet::depth_frames)
      .def_readonly("timestamp", &rcs::sim::FrameSet::timestamp);
  py::class_<rcs::sim::SimCameraSet>(sim, "SimCameraSet")
      .def(py::init<std::shared_ptr<rcs::sim::Sim>,
                    std::unordered_map<std::string, rcs::sim::SimCameraConfig>,
                    bool, int>(),
           py::arg("sim"), py::arg("cameras"),
           py::arg("render_on_demand") = true,
           py::arg("max_buffer_frames") = 100)
      .def("buffer_size", &rcs::sim::SimCameraSet::buffer_size)
      .def("clear_buffer", &rcs::sim::SimCameraSet::clear_buffer)
      .def("get_latest_frameset", &rcs::sim::SimCameraSet::get_latest_frameset)
      .def_property_readonly("_sim", &rcs::sim::SimCameraSet::get_sim)
      .def("get_timestamp_frameset",
           &rcs::sim::SimCameraSet::get_timestamp_frameset, py::arg("ts"));
  py::class_<rcs::sim::GuiClient>(sim, "GuiClient")
      .def(py::init<const std::string&>(), py::arg("id"))
      .def("get_model_bytes",
           [](const rcs::sim::GuiClient& self) {
             auto s = self.get_model_bytes();
             return py::bytes(s);
           })
      .def("set_model_and_data",
           [](rcs::sim::GuiClient& self, long m, long d) {
             self.set_model_and_data((mjModel*)m, (mjData*)d);
           })
      .def("sync", &rcs::sim::GuiClient::sync);
}
