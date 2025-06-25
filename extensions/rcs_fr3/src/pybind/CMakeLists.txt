pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE hw rcs mdl)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(franka PROPERTIES
    INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../rcs"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
# in pip
install(TARGETS _core franka DESTINATION rcs_fr3 COMPONENT python_package)
