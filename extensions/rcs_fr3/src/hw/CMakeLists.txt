add_library(hw)
target_sources(hw PRIVATE Franka.cpp FrankaHand.cpp FrankaMotionGenerator.cpp)
target_link_libraries(
  hw
  PUBLIC franka rcs pinocchio::all Eigen3::Eigen MuJoCo::MuJoCo
)

# add_executable(test_hw main.cpp)
# target_link_libraries(test_hw PRIVATE hw)
