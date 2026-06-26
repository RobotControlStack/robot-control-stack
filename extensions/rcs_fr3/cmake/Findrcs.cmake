if (NOT rcs_FOUND)
    if (NOT Python3_FOUND)
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs 1. Please install rcs-core using pip.")
        endif()
        return()
    endif()

    execute_process(
        COMMAND
            ${Python3_EXECUTABLE}
            -c
            "import importlib.util; import pathlib; spec = importlib.util.find_spec('rcs'); print(pathlib.Path(next(iter(spec.submodule_search_locations))).resolve() if spec and spec.submodule_search_locations else '')"
        OUTPUT_VARIABLE rcs_package_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (NOT rcs_package_dir)
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs 2. Please install rcs-core using pip.")
        endif()
        return()
    endif()

    # Check if the include directory exists
    cmake_path(APPEND rcs_package_dir include OUTPUT_VARIABLE rcs_INCLUDE_DIRS)
    if (NOT EXISTS ${rcs_INCLUDE_DIRS})
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs 3. Please install rcs-core using pip.")
        endif()
        return()
    endif()

    # Check if the library file exists
    set(rcs_library_path "${rcs_package_dir}/librcs.so")
    if (NOT EXISTS ${rcs_library_path})
        set(rcs_FOUND FALSE)
        if (rcs_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find rcs 4. Please install rcs-core using pip.")
        endif()
        return()
    endif()

    # Extract version from the library filename
    # file(GLOB rcs_dist_info "${Python3_SITELIB}/rcs-*.dist-info")
    # cmake_path(GET rcs_dist_info FILENAME rcs_library_filename)
    # string(REPLACE "rcs-" "" rcs_VERSION "${rcs_library_filename}")
    # string(REPLACE ".dist-info" "" rcs_VERSION "${rcs_VERSION}")

    # Create the imported target
    add_library(rcs SHARED IMPORTED)
    target_include_directories(rcs INTERFACE ${rcs_INCLUDE_DIRS})
    set_target_properties(
        rcs
        PROPERTIES
        IMPORTED_LOCATION "${rcs_library_path}"
    )
    set(rcs_FOUND TRUE)
endif()
