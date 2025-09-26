pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE rcs mdl pinocchio::all)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../rcs;$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)

# in pip
install(TARGETS _core DESTINATION rcs_robotics_library COMPONENT python_package)
