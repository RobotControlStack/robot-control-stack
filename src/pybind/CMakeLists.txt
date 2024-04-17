execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gen_mujoco_bindings.sh ${mujoco_SOURCE_DIR} ${MUJOCO_VERSION}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND_ERROR_IS_FATAL ANY
)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/cmake)
set(BUILD_TESTING_OLD ${BUILD_TESTING})
set(BUILD_TESTING OFF)
add_subdirectory(mujoco)
list(POP_BACK CMAKE_MODULE_PATH)
set(BUILD_TESTING ${BUILD_TESTING_OLD})

pybind11_add_module(_core MODULE rcsss.cpp)
target_link_libraries(_core PRIVATE
    hw
    sim
    common
    errors_header
    structs_header
    functions_header
)

# This is passing in the version as a define
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# Put the shared object files of these targets in site-packages/rcsss
install(TARGETS _core _callbacks _constants _enums _errors _functions _render _rollout _simulate _structs DESTINATION rcsss COMPONENT python_package)

