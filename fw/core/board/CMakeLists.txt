add_subdirectory(cloudchaser)
add_subdirectory(frdm_k22f)
add_subdirectory(frdm_k66f)
add_subdirectory(twr_k65f180m)

if(TARGET fw_core_board_${FW_BOARD_NAME})
    add_library(fw_core_board ALIAS fw_core_board_${FW_BOARD_NAME})
endif()
