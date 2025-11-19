pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE rcs Eigen3::Eigen pinocchio::all)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(_core PROPERTIES
    INSTALL_RPATH "$ORIGIN;$ORIGIN/../rcs;$ORIGIN/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)

# in pip
install(TARGETS _core DESTINATION rcs_so101 COMPONENT python_package)
