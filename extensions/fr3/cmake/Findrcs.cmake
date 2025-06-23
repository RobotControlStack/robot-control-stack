if (NOT rcs_FOUND)
    if (NOT Python3_FOUND)
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs. Please install rcs using pip 1.")
        endif()
        return()
    endif()

    # Check if the include directory exists
    cmake_path(APPEND Python3_SITELIB rcs include OUTPUT_VARIABLE rcs_INCLUDE_DIRS)
    if (NOT EXISTS ${rcs_INCLUDE_DIRS})
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs. Please install rcs using pip 2.")
        endif()
        return()
    endif()

    # Check if the library file exists
    cmake_path(APPEND Python3_SITELIB rcs OUTPUT_VARIABLE rcs_library_path)
    file(GLOB rcs_library_path "${rcs_library_path}/librcs.so")
    if (NOT EXISTS ${rcs_library_path})
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs. Please install rcs using pip 3.")
        endif()
        return()
    endif()

    # Extract version from the library filename
    file(GLOB rcs_dist_info "${Python3_SITELIB}/rcs-*.dist-info")
    cmake_path(GET rcs_dist_info FILENAME rcs_library_filename)
    string(REPLACE "rcs-" "" rcs_VERSION "${rcs_library_filename}")
    string(REPLACE ".dist-info" "" rcs_VERSION "${rcs_VERSION}")

    # Create the imported target
    add_library(rcs::rcs SHARED IMPORTED)
    target_include_directories(rcs::rcs INTERFACE ${rcs_INCLUDE_DIRS})
    set_target_properties(
        rcs::rcs
        PROPERTIES
        IMPORTED_LOCATION "${rcs_library_path}"
    )
    set(rcs_FOUND TRUE)
endif()