if (NOT MuJoCo_FOUND)
    if (NOT Python3_FOUND)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Check if the include directory exists
    cmake_path(APPEND Python3_SITELIB mujoco include OUTPUT_VARIABLE MuJoCo_INCLUDE_DIRS)
    if (NOT EXISTS ${MuJoCo_INCLUDE_DIRS})
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo. Please install MuJoCo using pip.")
        endif()
        return()
    endif()

    # Check if the library file exists
    cmake_path(APPEND Python3_SITELIB mujoco OUTPUT_VARIABLE mujoco_library_path)
    file(GLOB mujoco_library_path "${mujoco_library_path}/libmujoco.so.*")
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
