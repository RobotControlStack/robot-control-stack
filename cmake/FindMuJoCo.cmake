if (NOT MuJoCo_FOUND)
    if (NOT Python3_FOUND)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Run Python command to locate MuJoCo
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import mujoco; print(mujoco.__file__)" 
        OUTPUT_VARIABLE MUJOCO_FIND_OUTPUT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE result
    )

    # Check if the Python command was successful
    if (NOT result EQUAL 0)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Get the include directory path
    cmake_path(GET MUJOCO_FIND_OUTPUT PARENT_PATH MuJoCo_INCLUDE_DIRS)
    cmake_path(APPEND MuJoCo_INCLUDE_DIRS include) 

    # Check if the include directory exists
    if (NOT EXISTS ${MuJoCo_INCLUDE_DIRS})
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Locate the MuJoCo library
    cmake_path(GET MUJOCO_FIND_OUTPUT PARENT_PATH mujoco_library_path)
    file(GLOB mujoco_library_path "${mujoco_library_path}/libmujoco.so.*")

    # Check if the library was found
    if (NOT mujoco_library_path)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Extract version from the library filename
    cmake_path(GET mujoco_library_path FILENAME mujoco_library_filename)
    string(REPLACE "libmujoco.so." "" MuJoCo_VERSION "${mujoco_library_filename}")

    # Create the imported target
    add_library(MuJoCo::MuJoCo SHARED IMPORTED)
    target_include_directories(MuJoCo::MuJoCo INTERFACE ${MuJoCo_INCLUDE_DIRS})
    set_target_properties(
        MuJoCo::MuJoCo
        PROPERTIES
        IMPORTED_LOCATION "${mujoco_library_path}"
    )

    set(MuJoCo_FOUND TRUE)
endif()
