# add_library(mg STATIC motion_generator.cpp motion_generator.h fr3.h fr3.cpp)
add_library(mg SHARED motion_generator.cpp motion_generator.h fr3.h fr3.cpp)
target_include_directories(mg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Eigen3 REQUIRED)
target_link_libraries(mg PUBLIC Eigen3::Eigen franka)

add_subdirectory(hal)
add_subdirectory(pybind)

add_executable(main main.cpp)
target_link_libraries(main franka mg)
