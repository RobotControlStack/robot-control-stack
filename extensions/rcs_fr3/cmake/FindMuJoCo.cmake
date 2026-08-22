if (NOT MuJoCo_FOUND)
    if (NOT Python3_FOUND)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip 1.")
        endif()
        return()
    endif()

    # Get MuJoCo path from python
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import mujoco; print(mujoco.__path__[0])"
        OUTPUT_VARIABLE MUJOCO_PATH
        ERROR_VARIABLE MUJOCO_PYTHON_ERROR
        RESULT_VARIABLE MUJOCO_PYTHON_RESULT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if (MUJOCO_PYTHON_RESULT)
        message(STATUS "Python command failed with result: ${MUJOCO_PYTHON_RESULT}")
        message(STATUS "Python command stderr: ${MUJOCO_PYTHON_ERROR}")
    endif()

    if (NOT MUJOCO_PATH)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. MUJOCO_PATH is empty. Python command output: '${MUJOCO_PATH}'. Python command error: '${MUJOCO_PYTHON_ERROR}'. Please install MuJoCo using pip 2.")
        endif()
        return()
    endif()

    set(MuJoCo_INCLUDE_DIRS "${MUJOCO_PATH}/include")
    if (NOT EXISTS ${MuJoCo_INCLUDE_DIRS})
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip 3.")
        endif()
        return()
    endif()

    file(GLOB mujoco_library_path "${MUJOCO_PATH}/libmujoco.so.*")
    if (NOT mujoco_library_path)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip 4.")
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
