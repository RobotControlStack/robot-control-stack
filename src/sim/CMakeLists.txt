set(SIM_MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(plugin.h.in ${CMAKE_CURRENT_SOURCE_DIR}/plugin.h) # path to mujoco plugins
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

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
add_library(simulation STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
target_sources(simulation
    PUBLIC sim.h
    PRIVATE sim.cpp
    )
target_include_directories(simulation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(simulation PUBLIC platform_ui_adapter mujoco mdl)

add_executable(test_sim main.cpp)
target_link_libraries(test_sim PRIVATE simulation)

add_library(sim SHARED FR3.h FR3.cpp)
target_link_libraries(sim PUBLIC Eigen3::Eigen hal math mdl common mujoco)
