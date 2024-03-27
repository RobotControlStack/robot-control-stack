set(SIM_MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(plugin.h.in ${CMAKE_CURRENT_SOURCE_DIR}/plugin.h) # path to mujoco plugins

add_library(platform_ui_adapter)
target_sources(platform_ui_adapter
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_link_libraries(platform_ui_adapter PUBLIC mujoco glfw)

add_library(sim)
target_sources(sim PRIVATE sim.cpp)
target_link_libraries(sim PUBLIC common mujoco mdl platform_ui_adapter)

add_executable(test_sim main.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE mujoco mdl platform_ui_adapter)
