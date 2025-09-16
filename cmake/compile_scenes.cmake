include(FetchContent)

file(GLOB scene_dirs ${CMAKE_SOURCE_DIR}/assets/scenes/*)

# List to keep track of all generated .mjb files
set(mjb_files)

foreach(scene_dir ${scene_dirs})
    cmake_path(GET scene_dir FILENAME scene_name)
    set(scene_mjb ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}/scene.mjb)

    # Define the dependencies: all XML files and everything in the assets subdirectory
    file(GLOB_RECURSE xml_files ${scene_dir}/*.xml)
    file(GLOB_RECURSE asset_files ${scene_dir}/assets/*)

    # Add a custom command to compile the scene
    add_custom_command(
        OUTPUT ${scene_mjb}
        COMMAND ${Python_EXECUTABLE} -c "import mujoco as mj; mj.mj_saveModel(mj.MjModel.from_xml_path('${scene_dir}/scene.xml'), '${scene_mjb}')"
        DEPENDS ${xml_files} ${asset_files}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}
        COMMENT "Compiling scene ${scene_name}"
        VERBATIM
        ERROR_STRIP_TRAILING_WHITESPACE
    )

    # Add the generated .mjb file to the list
    list(APPEND mjb_files ${scene_mjb})

    # Install scene files
    install(FILES ${scene_mjb} DESTINATION rcs/scenes/${scene_name} COMPONENT python_package)

    # Copy all .xml files (dereferencing symlinks) and install them
    file(GLOB scene_xml_files ${scene_dir}/*.xml)
    foreach(xml_file ${scene_xml_files})
        get_filename_component(xml_name ${xml_file} NAME)
        configure_file(${xml_file} ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}/${xml_name} COPYONLY)
        install(FILES ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}/${xml_name} DESTINATION rcs/scenes/${scene_name} COMPONENT python_package)
    endforeach()

    # Always copy robot.xml to build dir, dereferencing symlinks
    if(EXISTS ${scene_dir}/robot.xml)
        configure_file(${scene_dir}/robot.xml ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}/robot.xml COPYONLY)
        install(FILES ${CMAKE_BINARY_DIR}/assets/scenes/${scene_name}/robot.xml DESTINATION rcs/scenes/${scene_name} COMPONENT python_package)
    endif()
    if(EXISTS ${scene_dir}/robot.urdf)
        install(FILES ${scene_dir}/robot.urdf DESTINATION rcs/scenes/${scene_name} COMPONENT python_package)
    endif()
    if(EXISTS ${scene_dir}/scene.xml)
        install(FILES ${scene_dir}/scene.xml DESTINATION rcs/scenes/${scene_name} COMPONENT python_package)
    endif()

endforeach()

# Create a custom target that depends on all generated .mjb files
add_custom_target(scenes ALL DEPENDS ${mjb_files})
