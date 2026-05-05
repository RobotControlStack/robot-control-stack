if (APPLE)
    set(_rpath_origin "@loader_path")
else()
    set(_rpath_origin "$ORIGIN")
endif()

pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE hw rcs pinocchio::all)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(franka PROPERTIES
    INSTALL_RPATH "${_rpath_origin}/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
set_target_properties(_core PROPERTIES
    INSTALL_RPATH "${_rpath_origin};${_rpath_origin}/../rcs;${_rpath_origin}/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)
# in pip
install(TARGETS _core franka DESTINATION rcs_panda COMPONENT python_package)
