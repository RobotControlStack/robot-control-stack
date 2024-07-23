include(FetchContent)
function(compile_scenes)
    FetchContent_GetProperties(scenes SOURCE_DIR src)
    file(GLOB scenes ${src}/scenes/*)
    foreach(scene ${scenes})
        cmake_path(GET scene FILENAME scene_name)
        file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/scenes/${scene_name})
        file(GLOB_RECURSE urdfs ${scene}/*.urdf)
        foreach(urdf ${urdfs})
            install(FILES ${urdf} DESTINATION rcsss/scenes/${scene_name} COMPONENT python_package)
        endforeach()
        cmake_path(APPEND scene "scene.xml")
        message("Compiling scene ${scene_name}")
        execute_process(
            COMMAND
            python -c "import mujoco as mj; mj.mj_saveModel(mj.MjModel.from_xml_path('${scene}'), '${CMAKE_BINARY_DIR}/scenes/${scene_name}/scene.mjb')"
            COMMAND_ECHO STDOUT
        )
        install(FILES ${CMAKE_BINARY_DIR}/scenes/${scene_name}/scene.mjb DESTINATION rcsss/scenes/${scene_name} COMPONENT python_package)
    endforeach()
endfunction()
