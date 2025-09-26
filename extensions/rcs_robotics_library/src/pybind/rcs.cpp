#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "RL.h"
#include "rcs/Kinematics.h"
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
  auto rl = m.def_submodule("rl", "rcs robotics library module");

  py::object kinematics =
      (py::object)py::module_::import("rcs").attr("common").attr("Kinematics");
  py::class_<rcs::robotics_library::RoboticsLibraryIK,
             std::shared_ptr<rcs::robotics_library::RoboticsLibraryIK>>(
      rl, "RoboticsLibraryIK", kinematics)
      .def(py::init<const std::string &, size_t>(), py::arg("urdf_path"),
           py::arg("max_duration_ms") = 300);
}
