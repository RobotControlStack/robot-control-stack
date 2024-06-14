find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
set(MUJOCO_DIR ${Python_SITELIB}/mujoco)
set(MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

if (NOT (DEFINED MUJOCO_VERSION))
    message(FATAL_ERROR "please define MUJOCO_VERSION (eg. cmake -DMUJOCO_VERSION=3.1.5)")
endif()

set(MUJOCO_LIBRARY_PATH ${MUJOCO_DIR}/libmujoco.so.${MUJOCO_VERSION})
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)

# Install mujoco at configure time so cmake can find the headers and the dll
execute_process(COMMAND python -m pip install --upgrade mujoco==${MUJOCO_VERSION} COMMAND_ERROR_IS_FATAL ANY)

add_library(mujoco SHARED IMPORTED)
target_include_directories(mujoco INTERFACE ${MUJOCO_INCLUDE_DIRS})
set_target_properties(
    mujoco
    PROPERTIES
    IMPORTED_LOCATION ${MUJOCO_LIBRARY_PATH}
)

add_library(sim)
target_sources(sim
  PRIVATE sim.cpp FR3.cpp
)
target_link_libraries(sim PUBLIC mujoco)

target_sources(sim PRIVATE FR3.cpp)
target_link_libraries(sim PUBLIC common mujoco mdl)

add_executable(test_sim test.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE sim mujoco common mdl)
