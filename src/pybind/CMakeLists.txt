execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gen_mujoco_bindings.sh ${mujoco_SOURCE_DIR} ${MUJOCO_VERSION} ${CMAKE_SOURCE_DIR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND_ERROR_IS_FATAL ANY
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/cmake)
set(BUILD_TESTING_OLD ${BUILD_TESTING})
set(BUILD_TESTING OFF)
add_subdirectory(mujoco)
list(POP_BACK CMAKE_MODULE_PATH)
set(BUILD_TESTING ${BUILD_TESTING_OLD})

set_target_properties(
  _callbacks
  _constants
  _enums
  _errors
  _functions
  _render
  _rollout
  _simulate
  _structs
  PROPERTIES
  INSTALL_RPATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

pybind11_add_module(_core MODULE rcsss.cpp)
target_link_libraries(_core PRIVATE hw sim common)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION rcsss COMPONENT python_package)
install(
  TARGETS 
  _callbacks
  _constants
  _enums
  _errors
  _functions
  _render
  _rollout
  _simulate
  _structs
  DESTINATION mujoco
  COMPONENT python_package
)
