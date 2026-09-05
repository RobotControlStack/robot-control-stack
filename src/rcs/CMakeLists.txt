if (APPLE)
    set(_rpath_origin "@loader_path")
else()
    set(_rpath_origin "$ORIGIN")
endif()

add_library(rcs SHARED)
target_include_directories(rcs PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_sources(rcs PRIVATE Pose.cpp Robot.cpp Kinematics.cpp utils.cpp)
target_link_libraries(rcs PUBLIC Eigen3::Eigen pinocchio::all)
set_target_properties(rcs PROPERTIES
    INSTALL_RPATH "${_rpath_origin};${_rpath_origin}/../cmeel.prefix/lib"
)
