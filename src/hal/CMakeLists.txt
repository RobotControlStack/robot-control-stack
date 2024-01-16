find_package(Eigen3 REQUIRED)
add_library(ha STATIC FR3.cpp FR3.h FrankaHand.cpp FrankaHand.h)
target_include_directories(ha PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ha PUBLIC Eigen3::Eigen franka mg hal math mdl)

add_executable(test_ha test_hal.cpp)
target_link_libraries(test_ha ha)