#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "SO101.h"
#include "rcs/Kinematics.h"

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
  auto ik = m.def_submodule("so101_ik", "rcs relaxed ik for so101");

  py::object kinematics =
      (py::object)py::module_::import("rcs").attr("common").attr("Kinematics");
  py::class_<rcs::so101::SO101IK,
             std::shared_ptr<rcs::so101::SO101IK>>(
      ik, "SO101IK", kinematics)
      .def(py::init<const std::string &, const std::string &, bool>(), py::arg("path"),
           py::arg("frame_id"), py::arg("urdf") = true);
}
