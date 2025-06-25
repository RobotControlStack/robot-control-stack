add_library(common)
target_sources(common PRIVATE Pose.cpp Robot.cpp IK.cpp utils.cpp)
target_link_libraries(common PUBLIC Eigen3::Eigen mdl pinocchio::pinocchio)
