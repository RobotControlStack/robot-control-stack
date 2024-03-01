#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// TODO: shouldnt this be .h, but only .cpp works?
#include "hw/FR3.cpp"
#include "hw/FrankaHand.cpp"

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

  // py::class_<rcs::hw::FR3>(m, "FR3")
  //     .def(py::init<const std::string &, const std::string &>(),
  //     py::arg("ip"),
  //          py::arg("filename"))
  //     .def("set_joint_position", &rcs::hw::FR3::set_joint_position,
  //     py::arg("q")) .def("get_joint_position",
  //     &rcs::hw::FR3::get_joint_position) .def("get_cartesian_position",
  //     &rcs::hw::FR3::get_cartesian_position) .def("set_guiding_mode",
  //     &rcs::hw::FR3::set_guiding_mode, py::arg("enabled")) .def("move_home",
  //     &rcs::hw::FR3::move_home) .def("automatic_error_recovery",
  //     &rcs::hw::FR3::automatic_error_recovery)
  //     .def("double_tap_robot_to_continue",
  //     &rcs::hw::FR3::double_tap_robot_to_continue) .def("set_parameters",
  //     &rcs::hw::FR3::set_parameters, py::arg("speed_factor"),
  //     py::arg("load_parameters")) .def("set_cartesian_position",
  //     &rcs::hw::FR3::move_cartesian, py::arg("x"),
  //          py::arg("controller"), py::arg("nominal_end_effector_frame") =
  //          py::none());

  // py::class_<rcs::hw::FrankaHand>(m, "FrankaHand")
  //     .def(py::init<const std::string &>(), py::arg("ip"))
  //     .def("start", &rcs::hw::FrankaHand::start)
  //     .def("stop", &rcs::hw::FrankaHand::stop)
  //     .def("setParameters", &rcs::hw::FrankaHand::setParameters,
  //          py::arg("grapsing_width"), py::arg("speed"), py::arg("force"))
  //     .def("getState", &rcs::hw::FrankaHand::getState)
  //     .def("halt", &rcs::hw::FrankaHand::halt)
  //     .def("release", &rcs::hw::FrankaHand::release)
  //     .def("shut", &rcs::hw::FrankaHand::shut);

  auto common = m.def_submodule("common", "common module");
  py::class_<rcs::common::Pose>(common, "Pose")
      .def(py::init<>())
      .def(py::init<const Eigen::Matrix4d &>(), py::arg("pose"))
      .def(py::init<const Eigen::Matrix3d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const Eigen::Vector4d &, const Eigen::Vector3d &>(),
           py::arg("rotation"), py::arg("translation"))
      .def(py::init<const rcs::common::RPY &, const Eigen::Vector3d &>(),
           py::arg("rpy"), py::arg("translation"))
      .def("translation", &rcs::common::Pose::translation)
      .def("rotation_m", &rcs::common::Pose::rotation_m)
      .def("rotation_q", &rcs::common::Pose::rotation_q)
      .def("pose_matrix", &rcs::common::Pose::pose_matrix)
      .def("rpy", &rcs::common::Pose::rpy)
      .def("interpolate", &rcs::common::Pose::interpolate, py::arg("dest_pose"),
           py::arg("progress"))
      .def("__repr__", &rcs::common::Pose::str)
      .def(py::self + py::self);

  py::class_<rcs::common::RPY>(common, "RPY")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("roll"),
           py::arg("pitch"), py::arg("yaw"))
      .def_readwrite("roll", &rcs::common::RPY::roll)
      .def_readwrite("pitch", &rcs::common::RPY::pitch)
      .def_readwrite("yaw", &rcs::common::RPY::yaw)
      .def("__repr__", &rcs::common::RPY::str)
      .def(py::self + py::self);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
