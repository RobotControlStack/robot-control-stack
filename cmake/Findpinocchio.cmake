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

    cmake_path(APPEND Python3_SITELIB cmeel.prefix lib OUTPUT_VARIABLE pinocchio_LIBRARY_DIR)

    set(_pinocchio_default_globs)
    set(_pinocchio_parsers_globs)
    if (APPLE)
        list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_default*.dylib")
        list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers*.dylib")
    elseif (WIN32)
        list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/pinocchio_default*.dll")
        list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_default*.dll")
        list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/pinocchio_parsers*.dll")
        list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers*.dll")
    else()
        list(APPEND _pinocchio_default_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_default.so*")
        list(APPEND _pinocchio_parsers_globs "${pinocchio_LIBRARY_DIR}/libpinocchio_parsers.so*")
    endif()

    file(GLOB pinocchio_library_paths LIST_DIRECTORIES FALSE ${_pinocchio_default_globs})
    list(LENGTH pinocchio_library_paths _pinocchio_library_count)
    if (_pinocchio_library_count EQUAL 0)
        set(pinocchio_FOUND FALSE)
        if (pinocchio_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find pinocchio library. Searched: ${_pinocchio_default_globs}")
        endif()
        return()
    endif()
    list(GET pinocchio_library_paths 0 pinocchio_library_path)

    file(GLOB pinocchio_parsers_paths LIST_DIRECTORIES FALSE ${_pinocchio_parsers_globs})
    list(LENGTH pinocchio_parsers_paths _pinocchio_parsers_count)
    if (_pinocchio_parsers_count EQUAL 0)
        set(pinocchio_FOUND FALSE)
        if (pinocchio_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find pinocchio parsers library. Searched: ${_pinocchio_parsers_globs}")
        endif()
        return()
    endif()
    list(GET pinocchio_parsers_paths 0 pinocchio_parsers_path)

    # Extract version from the library filename
    file(GLOB pinocchio_dist_info "${Python3_SITELIB}/pin-*.dist-info")
    cmake_path(GET pinocchio_dist_info FILENAME pinocchio_library_filename)
    string(REPLACE "pin-" "" pinocchio_VERSION "${pinocchio_library_filename}")
    string(REPLACE ".dist-info" "" pinocchio_VERSION "${pinocchio_VERSION}")

    # Create the imported target
    add_library(pinocchio::pinocchio SHARED IMPORTED)
    target_include_directories(pinocchio::pinocchio INTERFACE ${pinocchio_INCLUDE_DIRS})
    set_target_properties(pinocchio::pinocchio
        PROPERTIES
        IMPORTED_LOCATION "${pinocchio_library_path}"
    )

    add_library(pinocchio::parsers SHARED IMPORTED)
    target_include_directories(pinocchio::parsers INTERFACE ${pinocchio_INCLUDE_DIRS})
    set_target_properties(pinocchio::parsers
        PROPERTIES
        IMPORTED_LOCATION "${pinocchio_parsers_path}"
    )

    add_library(pinocchio::all INTERFACE IMPORTED)
    set_target_properties(pinocchio::all
        PROPERTIES
        INTERFACE_LINK_LIBRARIES "pinocchio::pinocchio;pinocchio::parsers"
    )
    set(pinocchio_FOUND TRUE)

endif()
