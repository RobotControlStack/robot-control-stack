cmake_minimum_required(VERSION 3.24)

project(
    rcs_fr3
    LANGUAGES C CXX
    VERSION 0.6.2
    DESCRIPTION "RCS Libfranka integration"
)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # Allow us to set options for subprojects

cmake_policy(SET CMP0048 NEW) # Set version in project
# Allow target properties affecting visibility during linking in static libraries
set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
cmake_policy(SET CMP0072 NEW) # Use GLVND instead of legacy libGL.so
cmake_policy(SET CMP0135 NEW) # Use timestamp of file extraction not download
cmake_policy(SET CMP0140 NEW) # Check return arguments

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# turn off libfranka tests
set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_PYTHON_INTERFACE OFF)
set(BUILD_DOCUMENTATION OFF)

include(FetchContent)

find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rcs REQUIRED)

FetchContent_Declare(
    libfranka
    GIT_REPOSITORY https://github.com/frankarobotics/libfranka.git
    GIT_TAG 0.20.3
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
    # Overwrite the version check script with our dummy script
    PATCH_COMMAND  ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SetVersionFromGit.cmake"
        "cmake/SetVersionFromGit.cmake"
)
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.4
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.1
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE
)
# --- tinyxml2 ---
FetchContent_Declare(
    tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG        10.0.0
    GIT_PROGRESS   TRUE
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE
)

# --- console_bridge ---
FetchContent_Declare(
    console_bridge
    GIT_REPOSITORY https://github.com/ros/console_bridge.git
    GIT_TAG        1.0.2
    GIT_PROGRESS   TRUE
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(pybind11 Eigen3 tinyxml2 console_bridge)
if(NOT TARGET Eigen3::Eigen3)
    add_library(Eigen3::Eigen3 ALIAS eigen)
endif()
if(NOT TARGET TinyXML2::TinyXML2)
    add_library(TinyXML2::TinyXML2 ALIAS tinyxml2)
endif()
if(NOT TARGET console_bridge::console_bridge)
    add_library(console_bridge::console_bridge ALIAS console_bridge)
endif()
FetchContent_MakeAvailable(libfranka)

add_subdirectory(src)
