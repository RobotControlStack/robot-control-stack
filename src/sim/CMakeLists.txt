add_library(sim)
target_sources(sim
  PRIVATE sim.cpp SimRobot.cpp renderer.cpp camera.cpp SimGripper.cpp gui_server.cpp gui_client.cpp
)
target_link_libraries(sim PUBLIC glfw common MuJoCo::MuJoCo mujoco_ui_lib)

add_executable(test_sim test.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE sim MuJoCo::MuJoCo common glfw)
