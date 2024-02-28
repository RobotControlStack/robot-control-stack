find_package(Eigen3 REQUIRED)
add_library(rcs::hw STATIC FR3.cpp FR3.h FrankaHand.cpp FrankaHand.h motion_generator.cpp motion_generator.h)
target_include_directories(rcs::hw PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rcs::hw PUBLIC Eigen3::Eigen franka mg hal math mdl)

add_executable(test_hw main.cpp)
target_link_libraries(test_hw rcs::hw)