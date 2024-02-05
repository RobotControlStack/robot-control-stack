cmake_minimum_required(VERSION 3.16)

project(
    franka_interface
    LANGUAGES C CXX
    VERSION 0.1.0
    DESCRIPTION "Franka Abstraction Layer"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW) # Set version in project
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW) # Use GLVND instead of legacy libGL.so
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # Allow us to set options for subprojects

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# turn off libfranka tests
set(BUILD_TESTS OFF)
set(RL_BUILD_DEMOS OFF)
set(RL_BUILD_RL_SG OFF)
set(RL_BUILD_TESTS OFF)
set(RL_BUILD_EXTRAS OFF)
set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_SIMULATE OFF)
set(MUJOCO_BUILD_TESTS OFF)
set(MUJOCO_TEST_PYTHON_UTIL OFF)

include(FetchContent)
FetchContent_Declare(
    libfranka
    GIT_REPOSITORY https://github.com/frankaemika/libfranka.git
    GIT_TAG 0.10.0
    GIT_PROGRESS TRUE
)
FetchContent_Declare(rl
    GIT_REPOSITORY https://github.com/roboticslibrary/rl.git
    GIT_TAG master
    GIT_PROGRESS TRUE
)
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG master
    GIT_PROGRESS TRUE
)
FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG 3.1.1
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(libfranka rl pybind11 mujoco)

add_subdirectory(src)
