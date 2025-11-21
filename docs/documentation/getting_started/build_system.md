# Build System

## 1. You Run

    pip install -e .

This tells `pip` to:

- Perform an **editable install** of the current project.
- Use the `pyproject.toml` as the **single source of truth** for building and packaging.

---

## 2. `pip` Reads `pyproject.toml`

It sees:

    [build-system]
    build-backend = "scikit_build_core.build"

So `pip` uses **`scikit-build-core`** as the **build backend** (instead of legacy `setuptools`).

---

## 3. `scikit-build-core` Invokes CMake

It starts a CMake build, just like if you had run:

    cmake -S . -B build
    cmake --build build

It uses the `CMakeLists.txt` at the project root, which:

- Declares the project:
  
      project(rcs LANGUAGES C CXX VERSION 0.4.0)
  
- Sets modern C++20 and compiler policies.
- Locates external dependencies:
  - `Eigen3`, `Python3`, `MuJoCo`, `pinocchio`
- Downloads with `FetchContent`: `rl`, `pybind11`
- Includes:
  
      add_subdirectory(src)

---

## 4. CMake Enters `src/CMakeLists.txt`

    add_subdirectory(rcs)
    target_include_directories(rcs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
    add_subdirectory(sim)
    add_subdirectory(pybind)

This loads and builds 3 subcomponents.

---

## 5. Subcomponent Builds

### `src/rcs/CMakeLists.txt`

    add_library(rcs SHARED)
    target_sources(rcs PRIVATE Pose.cpp Robot.cpp IK.cpp utils.cpp)
    target_link_libraries(rcs PUBLIC Eigen3::Eigen mdl pinocchio::all)

- Builds a **shared C++ library**: `rcs`
- Contains your **robot control logic**
- Exposes `include/` headers
- Linked against external libraries

### `src/sim/CMakeLists.txt`

    add_library(sim)
    target_sources(sim PRIVATE sim.cpp SimRobot.cpp ...)
    target_link_libraries(sim PUBLIC rcs MuJoCo::MuJoCo)

- Builds a **C++ simulation library**
- Depends on:
  - Your own `rcs` library
  - MuJoCo physics engine

### `src/pybind/CMakeLists.txt`

    pybind11_add_module(_core MODULE rcs.cpp)
    target_link_libraries(_core PRIVATE sim rcs)

- Compiles a Python extension module: `_core.so`
- Uses `pybind11` to bind C++ classes/functions
- Links to both `sim` and `rcs` native libraries
- Adds install instructions:
  
      install(TARGETS _core rcs DESTINATION rcs COMPONENT python_package)

---

## 6. Packaging into Python

From `pyproject.toml`:

    [tool.scikit-build]
    build.targets = ["_core", "scenes", "rcs"]
    wheel.packages = ["python/rcs"]
    install.components = ["python_package"]

- `scikit-build-core` installs `_core.so` into:
  
      python/rcs/_core.so
  
- That directory becomes a valid **Python package**

---

## 7. Result: Python Import Works

After install:

    from rcs import _core

- You now access the C++ functionality exposed in `rcs.cpp` through Python.
- `_core` contains Python-wrapped C++ objects/functions via `pybind11`.

---

## Summary Diagram

    pip install -e .
            │
            ▼
    Reads pyproject.toml (scikit_build_core used)
            │
            ▼
    CMakeLists.txt (root) → add_subdirectory(src)
            │
            ▼
    src/rcs → builds C++ library `rcs`
    src/sim → builds C++ library `sim`
    src/pybind → builds Python extension `_core`
            │
            ▼
    All .so libraries installed in `python/rcs/`
    `_core` accessible from Python as import

---

## Recap of Build Layers

| Layer | Role | Tool |
| --- | --- | --- |
| `pyproject.toml` | Project metadata, build backend | `pip`, `scikit-build-core` |
| `CMakeLists.txt` (root) | Configure the full C++ build | `CMake` |
| `src/rcs` | Core robot logic in C++ | `C++`, `Eigen`, `pinocchio` |
| `src/sim` | Simulation logic, MuJoCo | `C++`, `MuJoCo` |
| `src/pybind` | Bindings to Python | `pybind11` |
| `python/rcs/` | Python package with compiled `.so` | `scikit-build`, `pip` |



