# Source: https://gitlab.kitware.com/cmake/cmake/-/issues/21274
function(urlencode input output_variable)
    string(HEX "${input}" hex)
    string(LENGTH "${hex}" length)
    math(EXPR last "${length} - 1")
    set(result "")
    foreach(i RANGE ${last})
        math(EXPR even "${i} % 2")
        if("${even}" STREQUAL "0")
            string(SUBSTRING "${hex}" "${i}" 2 char)
            string(APPEND result "%${char}")
        endif()
    endforeach()
    set("${output_variable}" ${result} PARENT_SCOPE)
endfunction()

function(get_file_url file_path ref output)
    urlencode(${file_path} file_path_encoded)
    set("${output}" "https://gitos.rrze.fau.de/api/v4/projects/1100/repository/files/${file_path_encoded}/raw?ref=${ref}&lfs=true" PARENT_SCOPE)
endfunction()

function(download_file remote_path ref dst_path)
    get_file_url(${remote_path} ${ref} url)
    message("Downloading ${remote_path}")
    file(DOWNLOAD ${url}
         ${dst_path}
         HTTPHEADER "PRIVATE-TOKEN: ${GITLAB_MODELS_TOKEN}"
     )
endfunction()

function(download_assets_if_missing asset_remote_paths ref dst_dir)
    foreach(remote_path ${asset_remote_paths})
        cmake_path(GET remote_path FILENAME file_name)
        cmake_path(APPEND dst_dir ${file_name} OUTPUT_VARIABLE local_path)
        if (NOT (EXISTS ${local_path}))
            download_file(${remote_path} ${ref} ${local_path})
        endif()
    endforeach()
endfunction()
