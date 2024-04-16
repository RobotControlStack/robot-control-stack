pybind11_add_module(_core MODULE rcsss.cpp)
target_link_libraries(_core PRIVATE hw sim common)

# This is passing in the version as a define
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# Put the shared object files of these targets in site-packages/rcsss
install(TARGETS _core DESTINATION rcsss COMPONENT python_package)

execute_process(
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/mujoco_bindings.sh ${mujoco_SOURCE_DIR}
)
#include(${mujoco_SOURCE_DIR}/python/dist/mujoco-3.1.4/mujoco)
#message(INFO ${LIBRARIES_FOR_WHEEL})
