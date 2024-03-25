add_library(common SHARED Pose.cpp Pose.h utils.h Robot.h NRobotsWithGripper.cpp NRobotsWithGripper.h)
target_link_libraries(common PUBLIC Eigen3::Eigen)
