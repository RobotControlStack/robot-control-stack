# Creating a C++ Extension

For performance-critical hardware drivers or integration with C++ libraries, you can create a C++ extension. This involves writing C++ code and exposing it to Python using `pybind11`.

## Structure

A C++ extension typically looks like this:

```text
rcs_mycppext/
├── CMakeLists.txt
├── pyproject.toml
├── src/
│   ├── MyDevice.cpp
│   ├── MyDevice.h
│   └── bindings.cpp
└── ...
```

## Steps

1.  **CMake Configuration**: Use `CMakeLists.txt` to configure your build. You'll need to link against `rcs` (if it exposes C++ headers) and `pybind11`.

2.  **Implement C++ Class**: Write your device driver in C++.

    ```cpp
    #include <rcs/Robot.h>

    class MyRobot : public rcs::Robot {
    public:
        void setJointPosition(const Eigen::VectorXd& q) override {
            // ... implementation ...
        }
        // ... other methods ...
    };
    ```

3.  **Create Bindings**: Use `pybind11` to expose your class to Python.

    ```cpp
    #include <pybind11/pybind11.h>
    #include "MyDevice.h"

    namespace py = pybind11;

    PYBIND11_MODULE(rcs_mycppext, m) {
        py::class_<MyRobot, rcs::Robot>(m, "MyRobot")
            .def(py::init<>())
            .def("set_joint_position", &MyRobot::setJointPosition);
    }
    ```

4.  **Build System**: Use `scikit-build` or similar tools in `pyproject.toml` to compile the C++ extension during installation.

## Examples

- **rcs_fr3**: Implements the driver for the Franka Research 3 robot in C++ using `libfranka`.
- **rcs_robotics_library**: Wraps the Robotics Library (RL) for kinematics and path planning.
