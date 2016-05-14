if(NOT DEFINED CONFIG_FW_BOARD)
    set(CONFIG_FW_BOARD "cloudchaser" CACHE STRING "Target Firmware Board (default)")
endif()

add_subdirectory(${CONFIG_FW_BOARD})

add_library(fw_board ALIAS fw_board_${CONFIG_FW_BOARD})
