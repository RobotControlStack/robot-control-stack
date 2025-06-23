if (NOT pinocchio_FOUND)
    if (NOT Python3_FOUND)
        set(pinocchio_FOUND FALSE)
        if (pinocchio_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find pinocchio. Please install pinocchio using pip.")
        endif()
        return()
    endif()

    # Check if the include directory exists
    cmake_path(APPEND Python3_SITELIB cmeel.prefix include OUTPUT_VARIABLE pinocchio_INCLUDE_DIRS)
    if (NOT EXISTS ${pinocchio_INCLUDE_DIRS})
        set(pinocchio_FOUND FALSE)
        if (pinocchio_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find pinocchio. Please install pinocchio using pip.")
        endif()
        return()
    endif()

    # Check if the library file exists
    cmake_path(APPEND Python3_SITELIB cmeel.prefix lib libpinocchio.so OUTPUT_VARIABLE pinocchio_library_path)
    if (NOT EXISTS ${pinocchio_library_path})
        set(pinocchio_FOUND FALSE)
        if (pinocchio_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find pinocchio. Please install pinocchio using pip.")
        endif()
        return()
    endif()

    # Extract version from the library filename
    file(GLOB pinocchio_dist_info "${Python3_SITELIB}/pin-*.dist-info")
    cmake_path(GET pinocchio_dist_info FILENAME pinocchio_library_filename)
    string(REPLACE "pin-" "" pinocchio_VERSION "${pinocchio_library_filename}")
    string(REPLACE ".dist-info" "" pinocchio_VERSION "${pinocchio_VERSION}")

    # Create the imported target
    add_library(pinocchio::pinocchio SHARED IMPORTED)
    target_include_directories(pinocchio::pinocchio INTERFACE ${pinocchio_INCLUDE_DIRS})
    set_target_properties(
        pinocchio::pinocchio
        PROPERTIES
        IMPORTED_LOCATION "${pinocchio_library_path}"
    )
    set(pinocchio_FOUND TRUE)
endif()
