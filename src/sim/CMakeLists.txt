add_library(sim)
target_sources(sim
  PRIVATE sim.cpp FR3.cpp renderer.cpp camera.cpp FrankaHand.cpp gui_server.cpp gui_client.cpp
)
target_link_libraries(sim PUBLIC glfw common MuJoCo::MuJoCo mujoco_ui_lib)

add_executable(test_sim test.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE sim MuJoCo::MuJoCo common glfw)
