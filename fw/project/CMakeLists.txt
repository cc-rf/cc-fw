add_subdirectory(${FW_PROJECT_NAME})

if(TARGET ${FW_PROJECT_TARGET})
    add_library(fw_project ALIAS ${FW_PROJECT_TARGET})
endif()
