if (APPLE)
  set(_rpath_origin "@loader_path")
else()
  set(_rpath_origin "$ORIGIN")
endif()

pybind11_add_module(_core MODULE rcs.cpp)
target_link_libraries(_core PRIVATE sim rcs)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

set_target_properties(_core PROPERTIES
  INSTALL_RPATH "${_rpath_origin};${_rpath_origin}/../mujoco;${_rpath_origin}/../cmeel.prefix/lib"
    INTERPROCEDURAL_OPTIMIZATION TRUE
)

if (APPLE)
  get_target_property(_mujoco_library_path MuJoCo::MuJoCo IMPORTED_LOCATION)
  cmake_path(GET _mujoco_library_path FILENAME _mujoco_library_name)
  add_custom_command(TARGET _core POST_BUILD
    COMMAND install_name_tool
      -change "@rpath/mujoco.framework/Versions/A/${_mujoco_library_name}"
      "@loader_path/../mujoco/${_mujoco_library_name}"
      "$<TARGET_FILE:_core>"
    VERBATIM
  )
endif()

# in pip
install(TARGETS _core rcs DESTINATION rcs COMPONENT python_package)
install(
  DIRECTORY  ${CMAKE_SOURCE_DIR}/include/rcs
  DESTINATION  rcs/include
  COMPONENT python_package
)
