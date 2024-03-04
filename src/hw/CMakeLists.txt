find_package(Eigen3 REQUIRED)
add_library(hw STATIC FR3.cpp FR3.h FrankaHand.cpp FrankaHand.h MotionGenerator.cpp MotionGenerator.h)
target_include_directories(hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hw PUBLIC Eigen3::Eigen franka mg hal math mdl common)

add_executable(test_hw main.cpp)
target_link_libraries(test_hw hw)