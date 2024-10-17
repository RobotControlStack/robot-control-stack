add_library(hw)
target_sources(hw PRIVATE FR3.cpp FrankaHand.cpp MotionGenerator.cpp)
target_link_libraries(
  hw
  PUBLIC franka common
  PRIVATE math
)

add_executable(test_hw main.cpp)
target_link_libraries(test_hw PRIVATE hw)

add_executable(cartesian_impedance_control cartesian_impedance_control.cpp)
target_link_libraries(cartesian_impedance_control PRIVATE hw)

add_executable(generate_elbow_motion generate_elbow_motion.cpp)
target_link_libraries(generate_elbow_motion PRIVATE hw)
