
# Main Server(CONFIG_APP_PRODUCT_MODE=1)
# Test Server(CONFIG_APP_PRODUCT_MODE=0)
if(CONFIG_APP_PRODUCT_MODE)
    # define preprocessor macro with the same name
    add_compile_definitions(CONFIG_APP_PRODUCT_MODE=1)
else()
    add_compile_definitions(CONFIG_APP_PRODUCT_MODE=0)
endif()


# project info. display
function(app_proj_show product_mode)
    message("===========================================================")
    message("Project Name    ${PROJECT_NAME}")
    message("Version         ${PROJECT_VER}")

    set(build_mode "")
    if (${product_mode})
        set(build_mode "Main Server")
    else()
        set(build_mode "Test Server")
    endif()

    message("-----------------------------------------------------------")
endfunction()


# end of file
