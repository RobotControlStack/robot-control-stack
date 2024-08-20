cmake_minimum_required(VERSION 3.19)

project(
    rcs
    LANGUAGES C CXX
    VERSION 0.1.0
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

option(INCLUDE_UTN_MODELS "Whether to include the private UTN models. Requires GITLAB_MODELS_TOKEN to be set to a valid token wit read_api permissions" OFF)

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(MuJoCo REQUIRED)
find_package(pybind11 REQUIRED)

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

FetchContent_MakeAvailable(libfranka rl)

if (${INCLUDE_UTN_MODELS})
    if (DEFINED GITLAB_MODELS_TOKEN)
        include(download_scenes)
        set(ref 77e8782a3488cbf01361d2322f02e75c0d820644)
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
