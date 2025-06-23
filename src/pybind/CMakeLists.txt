pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE sim rcs)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../mujoco;$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
# in pip
install(TARGETS _core rcs DESTINATION rcs COMPONENT python_package)
