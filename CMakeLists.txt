cmake_minimum_required(VERSION 3.16)

project(
    franka_interface
    LANGUAGES C CXX
    VERSION 0.1.0
    DESCRIPTION "Franka Abstraction Layer"
)

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
FetchContent_MakeAvailable(libfranka rl)

add_subdirectory(src)
