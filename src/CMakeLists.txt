add_library(mg STATIC motion_generator.cpp motion_generator.h fr3.h fr3.cpp)

find_package(Eigen3 REQUIRED)
target_link_libraries(mg PUBLIC Eigen3::Eigen franka hal)

add_executable(main main.cpp)
target_link_libraries(main franka mg)
