add_library(common)
target_sources(common PRIVATE Pose.cpp NRobotsWithGripper.cpp)
target_link_libraries(common PUBLIC Eigen3::Eigen)
