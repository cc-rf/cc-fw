set(CONFIG_KINETIS_CPU "mk66f18" CACHE STRING "Target Kinetis CPU (cloudchaser)")

if(NOT DEFINED CONFIG_FW_BOARD_REV)
    set(CONFIG_FW_BOARD_REV "1" CACHE STRING "Target Firmware Board Revision (cloudchaser default)")
endif()

if(CONFIG_FW_BOARD_REV STREQUAL "2" )
    set(CONFIG_KINETIS_CPU_PART "MK66FN2M0VMD18" CACHE STRING "Target Kinetis CPU Part (cloudchaser)")
elseif(CONFIG_FW_BOARD_REV STREQUAL "1")
    set(CONFIG_KINETIS_CPU_PART "MK66FX1M0VMD18" CACHE STRING "Target Kinetis CPU Part (cloudchaser)")
else()
    message(FATAL_ERROR "Unknown CONFIG_FW_BOARD_REV ${CONFIG_FW_BOARD_REV}")
endif()
