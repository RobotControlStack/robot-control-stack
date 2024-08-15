pybind11_add_module(_core MODULE rcsss.cpp)
target_link_libraries(_core PRIVATE hw sim common)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../mujoco"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
install(TARGETS _core franka DESTINATION rcsss COMPONENT python_package)
