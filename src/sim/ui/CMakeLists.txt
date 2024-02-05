find_package(glfw3 REQUIRED)
add_library(
    ui
    STATIC
    ui.cpp
    ui.h
    glfw_adapter.cc
    glfw_adapter.h
    glfw_dispatch.h
    glfw_dispatch.cc
    platform_ui_adapter.cc
    platform_ui_adapter.h
)
target_link_libraries(ui PRIVATE glfw mujoco)
