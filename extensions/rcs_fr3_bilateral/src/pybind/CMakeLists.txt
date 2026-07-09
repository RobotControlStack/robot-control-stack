pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE bilateral_hw hw rcs pinocchio::all)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(franka PROPERTIES
    INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../rcs;$ORIGIN/../rcs_fr3;$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)

install(TARGETS _core franka DESTINATION rcs_fr3_bilateral COMPONENT python_package)
