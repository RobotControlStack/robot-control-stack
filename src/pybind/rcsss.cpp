#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// TODO: shouldnt this be .h, but only .cpp works?
#include "../hw/FR3.cpp"
#include "../hw/FrankaHand.cpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) { return i + j; }

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
  m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: scikit_build_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

  m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

  m.def(
      "subtract", [](int i, int j) { return i - j; },
      "pybind11 example plugin");

  py::class_<FR3>(m, "FR3")
      .def(py::init<const std::string &, const std::string &>(), py::arg("ip"),
           py::arg("filename"))
      .def("set_joint_position", &FR3::set_joint_position, py::arg("q"))
      .def("get_joint_position", &FR3::get_joint_position)
      .def("get_cartesian_position", &FR3::get_cartesian_position)
      .def("set_guiding_mode", &FR3::set_guiding_mode, py::arg("enabled"))
      .def("move_home", &FR3::move_home)
      .def("automatic_error_recovery", &FR3::automatic_error_recovery)
      .def("double_tap_robot_to_continue", &FR3::double_tap_robot_to_continue)
      .def("set_parameters", &FR3::setParameters, py::arg("speed_factor"), py::arg("load_parameters"))
      .def("set_cartesian_position", &FR3::move_cartesian, py::arg("x"),
           py::arg("controller"), py::arg("nominal_end_effector_frame") = py::none());

  py::class_<FrankaHand>(m, "FrankaHand")
      .def(py::init<const std::string &>(), py::arg("ip"))
      .def("start", &FrankaHand::start)
      .def("stop", &FrankaHand::stop)
      .def("setParameters", &FrankaHand::setParameters,
           py::arg("grapsing_width"), py::arg("speed"), py::arg("force"))
      .def("getState", &FrankaHand::getState)
      .def("halt", &FrankaHand::halt)
      .def("release", &FrankaHand::release)
      .def("shut", &FrankaHand::shut);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
