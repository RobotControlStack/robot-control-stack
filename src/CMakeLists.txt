add_subdirectory(rcs)
target_include_directories(rcs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(sim)
add_subdirectory(pybind)
