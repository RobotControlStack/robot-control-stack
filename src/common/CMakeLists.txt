add_library(rcs SHARED)
target_sources(rcs PRIVATE Pose.cpp Robot.cpp IK.cpp utils.cpp)
target_link_libraries(common PUBLIC Eigen3::Eigen mdl pinocchio::pinocchio pinocchio_parsers)
