if(NOT DEFINED CONFIG_FW_PROJECT)
    set(CONFIG_FW_PROJECT "default" CACHE STRING "Target Firmware Project (default)")
endif()

add_subdirectory(${CONFIG_FW_PROJECT})
