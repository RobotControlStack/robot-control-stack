if (NOT MuJoCo_FOUND)
    if (NOT ${Python3_FOUND})
        set(MuJoCo_FOUND FALSE)
        return()
    endif()
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import mujoco; print(mujoco.__file__)" 
        OUTPUT_VARIABLE MUJOCO_FIND_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    cmake_path(GET MUJOCO_FIND_OUTPUT PARENT_PATH MuJoCo_INCLUDE_DIRS)
    cmake_path(APPEND MuJoCo_INCLUDE_DIRS include) 

    if (NOT (EXISTS ${MuJoCo_INCLUDE_DIRS}))
        set(MuJoCo_FOUND FALSE)
        return()
    endif()

    set(MuJoCo_FOUND TRUE)
    cmake_path(GET MUJOCO_FIND_OUTPUT PARENT_PATH mujoco_library_path)
    file(GLOB mujoco_library_path "${mujoco_library_path}/libmujoco.so.*")
    cmake_path(GET mujoco_library_path FILENAME mujoco_library_filename)
    string(REPLACE "libmujoco.so." "" MuJoCo_VERSION "${mujoco_library_filename}")
    add_library(MuJoCo::MuJoCo SHARED IMPORTED)
    target_include_directories(MuJoCo::MuJoCo INTERFACE ${MuJoCo_INCLUDE_DIRS})
    set_target_properties(
        MuJoCo::MuJoCo
        PROPERTIES
        IMPORTED_LOCATION "${mujoco_library_path}"
    )
endif()
