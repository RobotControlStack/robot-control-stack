#ifndef RCS_PY_COPY_H
#define RCS_PY_COPY_H

// Copy support for value types bound with pybind11 (configs, states, ...),
// shared by the core and the extension bindings.

#include <pybind11/pybind11.h>

#include <cstdint>

namespace rcs {
namespace common {

namespace py = pybind11;

// Copies self, an instance of the bound C++ type T or of a Python subclass of
// it (e.g. rcs_fr3.hw.FR3Config): the C++ part with T's copy constructor, the
// attributes of a Python subclass (__dict__) shallow or, with a memo, deep.
template <typename T>
py::object copy_instance(py::handle self, py::object memo) {
  py::handle cls = reinterpret_cast<PyObject*>(Py_TYPE(self.ptr()));
  py::object result;
  if (cls.is(py::type::of<T>())) {
    result = py::cast(T(self.cast<const T&>()));
  } else {
    result = cls.attr("__new__")(cls);
    py::type::of<T>().attr("__init__")(result, self);  // the copy constructor
  }
  if (!memo.is_none()) {
    memo[py::int_(reinterpret_cast<std::uintptr_t>(self.ptr()))] = result;
  }
  if (py::hasattr(self, "__dict__")) {
    py::object attributes = self.attr("__dict__");
    if (!memo.is_none()) {
      attributes =
          py::module_::import("copy").attr("deepcopy")(attributes, memo);
    }
    result.attr("__dict__").attr("update")(attributes);
  }
  return result;
}

// Adds a copy constructor, __copy__ and __deepcopy__ to the bound value type
// T, registered as py::class_<T, Options...> under the given handle (e.g.
// m.attr("RobotConfig")). The C++ copy is a full copy, so the shallow and the
// deep copy of the C++ part are the same.
template <typename T, typename... Options>
void def_copy(py::handle registered) {
  py::reinterpret_borrow<py::class_<T, Options...>>(registered)
      .def(py::init<const T&>(), py::arg("other"), "Copy of other.")
      .def("__copy__",
           [](py::handle self) { return copy_instance<T>(self, py::none()); })
      .def("__deepcopy__", [](py::handle self, py::dict memo) {
        return copy_instance<T>(self, memo);
      });
}

}  // namespace common
}  // namespace rcs
#endif  // RCS_PY_COPY_H
