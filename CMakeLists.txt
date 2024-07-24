cmake_minimum_required(VERSION 3.19)

project(
    rcs
    LANGUAGES C CXX
    VERSION 0.1.0
    DESCRIPTION "UTNs Robot Control Stack Library"
)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

set(CMAKE_POLICY_DEFAULT_CMP0048 NEW) # Set version in project
# Allow target properties affecting visibility during linking in static libraries
set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW) # Use GLVND instead of legacy libGL.so
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW) # Allow us to set options for subprojects
set(CMAKE_POLICY_DEFAULT_CMP0135 NEW) # Use timestamp of file extraction not download

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# turn off libfranka tests
set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)
set(RL_BUILD_DEMOS OFF)
set(RL_BUILD_RL_SG OFF)
set(RL_BUILD_TESTS OFF)
set(RL_BUILD_EXTRAS OFF)

option(INCLUDE_UTN_MODELS "Whether to include the private UTN models. Requires GITLAB_MODELS_TOKEN to be set to a valid token with read_api permissions" OFF)

include(FetchContent)
FetchContent_Declare(
    libfranka
    GIT_REPOSITORY https://github.com/frankaemika/libfranka.git
    GIT_TAG 0.13.3
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(rl
    GIT_REPOSITORY https://github.com/roboticslibrary/rl.git
    GIT_TAG master
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG master
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)

FetchContent_MakeAvailable(libfranka rl pybind11)

if (${INCLUDE_UTN_MODELS})
    if (DEFINED GITLAB_MODELS_TOKEN)
      include(download_scenes)
      download_scenes(f31cde2879f4b39d1f8d3ae329dd10155727b352)
    else()
      message(FATAL_ERROR "GITLAB_MODELS_TOKEN missing. Not including lab scene")
    endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

add_subdirectory(src)
