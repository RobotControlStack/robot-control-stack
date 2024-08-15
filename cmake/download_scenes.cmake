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

function(download_scenes ref)
  FetchContent_Declare(
    scenes
    URL         "https://gitos.rrze.fau.de/api/v4/projects/1100/repository/archive?path=scenes&sha=${ref}"
    HTTP_HEADER "PRIVATE-TOKEN: ${GITLAB_MODELS_TOKEN}"
  )
  FetchContent_MakeAvailable(scenes)
  file(GLOB_RECURSE files ${scenes_SOURCE_DIR}/*)
  foreach(file ${files})
    if(IS_SYMLINK ${file})
      file(READ_SYMLINK ${file} symlink)
      string(REGEX REPLACE "^\.\.\/" "" remote_path ${symlink})
      file(REMOVE ${file})
      download_file(${remote_path} ${ref} ${file})
    endif()
  endforeach()
endfunction()
