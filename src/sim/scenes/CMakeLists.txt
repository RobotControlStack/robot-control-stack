if (DEFINED UTN_MODELS)
    if (DEFINED GITLAB_MODELS_TOKEN)
        # TODO: check for token validity and return a nice message in case it is not
        add_subdirectory(lab)
    else()
        message(WARNING "GITLAB_MODELS_TOKEN missing. Not including lab scene")
    endif()
endif()
