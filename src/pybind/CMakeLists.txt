# cmake_minimum_required(VERSION 3.15...3.27)

# project(
#   ${SKBUILD_PROJECT_NAME}
#   VERSION ${SKBUILD_PROJECT_VERSION}
#   LANGUAGES CXX)


# scikit-build
# find_package(Eigen3 REQUIRED)
# add_library(ha2 STATIC ../hal/FR3.cpp ../hal/FR3.h ../hal/FrankaHand.cpp ../hal/FrankaHand.h)
# target_include_directories(ha2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# # problem here is that mg is not available as a library
# target_link_libraries(ha2 PUBLIC Eigen3::Eigen franka mg hal math mdl)


# Find the module development requirements (requires FindPython from 3.17 or
# scikit-build-core's built-in backport)
# find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
# find_package(pybind11 CONFIG REQUIRED)

# Add a library using FindPython's tooling (pybind11 also provides a helper like
# this)
# python_add_library(_core MODULE pyfr3.cpp WITH_SOABI)
# target_link_libraries(_core PRIVATE pybind11::headers ha)

# plain pybind11 (no scikit-build)
pybind11_add_module(_core rcsss.cpp)
target_include_directories(_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(_core PUBLIC hw common)

# This is passing in the version as a define
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
install(TARGETS _core DESTINATION rcsss)