#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
// TODO: shouldnt this be .h, but only .cpp works?
#include "../hal/FR3.cpp"
#include "../hal/FrankaHand.cpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

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

    m.def("subtract", [](int i, int j) { return i - j; }, "pybind11 example plugin");

    py::class_<FR3>(m, "FR3")
        .def(py::init<const std::string &, const std::string &>(), py::arg("ip"), py::arg("filename"))
        .def("setJointPosition", &FR3::setJointPosition, py::arg("q"))
        .def("getJointPosition", &FR3::getJointPosition)
        .def("setGuidingMode", &FR3::setGuidingMode, py::arg("activated"), py::arg("enabled"))
        .def("move_home", &FR3::move_home)
        .def("setParameters", &FR3::setParameters, py::arg("speed_factor"));

    py::class_<FrankaHand>(m, "FrankaHand")
        .def(py::init<const std::string &>(), py::arg("ip"))
        .def("start", &FrankaHand::start)
        .def("stop", &FrankaHand::stop)
        .def("setParameters", &FrankaHand::setParameters, py::arg("grapsing_width"), py::arg("speed"), py::arg("force"))
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
