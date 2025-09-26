add_library(hw)
target_sources(hw PRIVATE Panda.cpp FrankaHand.cpp PandaMotionGenerator.cpp)
target_link_libraries(
  hw
  PUBLIC franka rcs pinocchio::all
  PRIVATE math
)

# add_executable(test_hw main.cpp)
# target_link_libraries(test_hw PRIVATE hw)
