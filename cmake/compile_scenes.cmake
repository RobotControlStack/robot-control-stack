include(FetchContent)
FetchContent_GetProperties(scenes SOURCE_DIR src)

file(GLOB scene_dirs ${src}/scenes/*)

# List to keep track of all generated .mjb files
set(mjb_files)

foreach(scene_dir ${scene_dirs})
    cmake_path(GET scene_dir FILENAME scene_name)
    set(scene_mjb ${CMAKE_BINARY_DIR}/scenes/${scene_name}/scene.mjb)
    set(scene_xml ${scene_dir}/scene.xml)

    # Create the output directory
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/scenes/${scene_name})

    # Define the dependencies: all XML files and everything in the assets subdirectory
    file(GLOB_RECURSE xml_files ${scene_dir}/*.xml)
    file(GLOB_RECURSE asset_files ${scene_dir}/assets/*)

    # Add a custom command to compile the scene
    add_custom_command(
        OUTPUT ${scene_mjb}
        COMMAND ${Python_EXECUTABLE} -c "import mujoco as mj; mj.mj_saveModel(mj.MjModel.from_xml_path('${scene_xml}'), '${scene_mjb}')"
        DEPENDS ${xml_files} ${asset_files}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scenes/${scene_name}
        COMMENT "Compiling scene ${scene_name}"
        VERBATIM
        ERROR_STRIP_TRAILING_WHITESPACE
    )

    # Add the generated .mjb file to the list
    list(APPEND mjb_files ${scene_mjb})

    # Install scene files
    install(FILES ${scene_mjb} DESTINATION rcsss/scenes/${scene_name} COMPONENT python_package)

    # Install URDF files
    file(GLOB_RECURSE urdfs ${scene_dir}/*.urdf)
    foreach(urdf ${urdfs})
        install(FILES ${urdf} DESTINATION rcsss/scenes/${scene_name} COMPONENT python_package)
    endforeach()

    # Install xml files
    foreach(xml ${xml_files})
        install(FILES ${xml} DESTINATION rcsss/scenes/${scene_name} COMPONENT python_package)
    endforeach()

    # Install assets
    foreach(asset ${asset_files})
        install(FILES ${asset} DESTINATION rcsss/scenes/${scene_name}/assets COMPONENT python_package)
    endforeach()
endforeach()

# Create a custom target that depends on all generated .mjb files
add_custom_target(utn_scenes ALL DEPENDS ${mjb_files})
