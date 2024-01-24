#include <pybind11/pybind11.h>
#include "../hal/FR3.cpp"

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
        .def(py::init<const std::string &, const std::string &>())
        .def("setJointPosition", &FR3::setJointPosition)
        .def("getJointPosition", &FR3::getJointPosition)
        .def("setGuidingMode", &FR3::setGuidingMode);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
