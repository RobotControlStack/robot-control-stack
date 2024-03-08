#include <common/NRobotsWithGripper.h>
#include <common/Pose.h>
#include <common/Robot.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

// TODO: define docstring in according python files

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

  py::class_<rcs::common::RConfig>(common, "RConfig");
  py::class_<rcs::common::RState>(common, "RState");
  py::class_<rcs::common::RState>(common, "GConfig");
  py::class_<rcs::common::GState>(common, "GState");

  // holder type should be smart pointer as we deal with smart pointer
  // instances of this class
  py::class_<rcs::common::Robot, rcs::common::PyRobot<>,
             std::shared_ptr<rcs::common::Robot>>(common, "Robot")
      .def(py::init<>())
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
      .def(py::init<>())
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

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
