find_package(Eigen3 REQUIRED)
# shared was needed for python bindings to compile with inheritance in attributes
add_library(hw SHARED FR3.cpp FR3.h FrankaHand.cpp FrankaHand.h MotionGenerator.cpp MotionGenerator.h)
target_include_directories(hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(hw PUBLIC Eigen3::Eigen franka hal math mdl common)

add_executable(test_hw main.cpp)
target_link_libraries(test_hw hw)
