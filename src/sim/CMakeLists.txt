set(SIM_MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(plugin.h.in ${CMAKE_CURRENT_SOURCE_DIR}/plugin.h) # path to mujoco plugins

add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_include_directories(
  platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC mujoco glfw)
set_target_properties(platform_ui_adapter PROPERTIES POSITION_INDEPENDENT_CODE True)

add_library(sim SHARED FR3.cpp FR3.h)
target_link_libraries(sim PUBLIC common mujoco mdl platform_ui_adapter)
add_executable(test_sim main.cpp sim.cpp)
target_link_libraries(test_sim PRIVATE sim)
