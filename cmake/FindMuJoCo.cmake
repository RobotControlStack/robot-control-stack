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

    set(_mujoco_library_globs)
    if (APPLE)
        list(APPEND _mujoco_library_globs "${MUJOCO_PATH}/libmujoco.*.dylib")
    elseif (WIN32)
        list(APPEND _mujoco_library_globs "${MUJOCO_PATH}/mujoco.dll")
        list(APPEND _mujoco_library_globs "${MUJOCO_PATH}/bin/mujoco.dll")
    else()
        list(APPEND _mujoco_library_globs "${MUJOCO_PATH}/libmujoco.so.*")
    endif()

    file(GLOB mujoco_library_paths LIST_DIRECTORIES FALSE ${_mujoco_library_globs})
    list(LENGTH mujoco_library_paths _mujoco_library_count)
    if (_mujoco_library_count EQUAL 0)
        set(MuJoCo_FOUND FALSE)
        if (MuJoCo_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find MuJoCo shared library. Searched: ${_mujoco_library_globs}")
        endif()
        return()
    endif()
    list(GET mujoco_library_paths 0 mujoco_library_path)

    # Extract version from the library filename
    cmake_path(GET mujoco_library_path FILENAME mujoco_library_filename)
    set(MuJoCo_VERSION "")
    if (mujoco_library_filename MATCHES "^libmujoco\\.so\\.(.+)$")
        set(MuJoCo_VERSION "${CMAKE_MATCH_1}")
    elseif (mujoco_library_filename MATCHES "^libmujoco\\.(.+)\\.dylib$")
        set(MuJoCo_VERSION "${CMAKE_MATCH_1}")
    endif()

    # Create the imported target
    add_library(MuJoCo::MuJoCo SHARED IMPORTED)
    target_include_directories(MuJoCo::MuJoCo INTERFACE ${MuJoCo_INCLUDE_DIRS})
    set_target_properties(
        MuJoCo::MuJoCo
        PROPERTIES
        IMPORTED_NO_SONAME ${APPLE}
        IMPORTED_LOCATION "${mujoco_library_path}"
    )

    set(MuJoCo_FOUND TRUE)
endif()
