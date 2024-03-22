add_library(hw SHARED)
target_sources(
  hw
  PUBLIC FR3.h FrankaHand.h MotionGenerator.h
  PRIVATE FR3.cpp FrankaHand.cpp MotionGenerator.cpp
)
target_link_libraries(
  hw
  PUBLIC franka mdl common
  PRIVATE hal math
)

add_executable(test_hw main.cpp)
target_link_libraries(test_hw hw)
