find_package(Eigen3 REQUIRED)
add_library(common STATIC Pose.cpp Pose.h utils.h)
target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(common PUBLIC Eigen3::Eigen hal pybind11::module)