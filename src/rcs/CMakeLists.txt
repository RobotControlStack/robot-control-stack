add_library(rcs SHARED)
target_include_directories(rcs PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_sources(rcs PRIVATE Pose.cpp Robot.cpp IK.cpp utils.cpp)
target_link_libraries(rcs PUBLIC Eigen3::Eigen mdl pinocchio::pinocchio pinocchio_parsers)
