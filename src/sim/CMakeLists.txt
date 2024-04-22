set(SIM_MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(plugin.h.in ${CMAKE_CURRENT_SOURCE_DIR}/plugin.h) # path to mujoco plugins

add_library(ui)
target_sources(ui
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_link_libraries(ui PUBLIC mujoco glfw)

add_library(sim)
target_sources(sim PRIVATE FR3.cpp)
target_link_libraries(sim PUBLIC common mujoco mdl ui)

add_executable(test_sim main.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE sim mujoco common mdl ui)
