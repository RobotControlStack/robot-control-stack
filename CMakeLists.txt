cmake_minimum_required(VERSION 3.19)

project(
    rcs
    LANGUAGES C CXX
    VERSION 0.3.1
    DESCRIPTION "UTNs Robot Control Stack Library"
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
set(BUILD_PYTHON_INTERFACE OFF)
set(BUILD_DOCUMENTATION OFF)

option(INCLUDE_UTN_MODELS "Whether to include the private UTN models. Requires GITLAB_MODELS_TOKEN to be set to a valid token wit read_api permissions" OFF)

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(MuJoCo REQUIRED)

FetchContent_Declare(
    libfranka
    GIT_REPOSITORY https://github.com/frankaemika/libfranka.git
    GIT_TAG 0.15.0
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(rl
    GIT_REPOSITORY https://github.com/roboticslibrary/rl.git
    GIT_TAG 0b3797215345a1d37903634095361233d190b2e6
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.4
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
set(MuJoCo_UI_URL https://raw.githubusercontent.com/google-deepmind/mujoco/${MuJoCo_VERSION}/simulate/)
set(MuJoCo_UI_FILES platform_ui_adapter.cc platform_ui_adapter.h glfw_adapter.cc glfw_adapter.h glfw_adapter.cc glfw_corevideo.h glfw_corevideo.mm glfw_dispatch.cc glfw_dispatch.h)
# Set the destination directory for the downloaded files
set(MuJoCo_UI_DIR "${CMAKE_BINARY_DIR}/mujoco_ui")

# Create the directory if it doesn't exist
file(MAKE_DIRECTORY ${MuJoCo_UI_DIR})

# Loop through each file and download it only if it doesn't already exist
foreach(file ${MuJoCo_UI_FILES})
  set(output_file "${MuJoCo_UI_DIR}/${file}")
  if (NOT EXISTS ${output_file})
    message(STATUS "Downloading ${file}...")
    file(DOWNLOAD ${MuJoCo_UI_URL}${file} ${output_file})
  else()
    message(STATUS "File ${file} already exists, skipping download.")
  endif()
endforeach()

# Add the include directories or link the files as needed
add_library(mujoco_ui_lib ${MuJoCo_UI_DIR}/glfw_adapter.cc ${MuJoCo_UI_DIR}/platform_ui_adapter.cc ${MuJoCo_UI_DIR}/glfw_dispatch.cc)
target_link_libraries(mujoco_ui_lib MuJoCo::MuJoCo glfw)
target_include_directories(mujoco_ui_lib INTERFACE ${MuJoCo_UI_DIR})

FetchContent_MakeAvailable(libfranka rl pybind11)

if (${INCLUDE_UTN_MODELS})
    if (DEFINED GITLAB_MODELS_TOKEN)
        include(download_scenes)
        set(ref bfdd70ae189ed3a925bf1c16d29162e847a21d56)
        FetchContent_Declare(
            scenes
            URL         "https://gitos.rrze.fau.de/api/v4/projects/1100/repository/archive?path=scenes&sha=${ref}"
            HTTP_HEADER "PRIVATE-TOKEN: ${GITLAB_MODELS_TOKEN}"
        )
        FetchContent_MakeAvailable(scenes)
        resolve_symlinks(${ref})
    else()
      message(FATAL_ERROR "GITLAB_MODELS_TOKEN missing. Not including lab scene")
    endif()
endif()
include(compile_scenes)

add_subdirectory(src)
