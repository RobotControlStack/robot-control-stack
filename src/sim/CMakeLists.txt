set(SIM_MODEL_DIR ${CMAKE_SOURCE_DIR}/models)
configure_file(plugin.h.in ${CMAKE_CURRENT_SOURCE_DIR}/plugin.h) # path to mujoco plugins

add_subdirectory(ik)
add_subdirectory(ui)

add_executable(sim main.c plugin.h sim.c sim.h)
target_link_libraries(sim PRIVATE mujoco ik ui)

add_library(libsim STATIC sim.c sim.h plugin.h)
target_link_libraries(libsim PRIVATE mujoco ik ui)
target_include_directories(libsim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
