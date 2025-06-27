add_library(hw)
target_sources(hw PRIVATE FR3.cpp FrankaHand.cpp FR3MotionGenerator.cpp)
target_link_libraries(
  hw
  PUBLIC franka rcs pinocchio::pinocchio pinocchio_parsers
  PRIVATE math
)

# add_executable(test_hw main.cpp)
# target_link_libraries(test_hw PRIVATE hw)
