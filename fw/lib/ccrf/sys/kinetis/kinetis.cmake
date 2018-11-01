set(
        CCRF_PLATFORM_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/amp.c
        ${CMAKE_CURRENT_LIST_DIR}/clock.c
        ${CMAKE_CURRENT_LIST_DIR}/isr.c
        ${CMAKE_CURRENT_LIST_DIR}/kinetis.c
        ${CMAKE_CURRENT_LIST_DIR}/spi.c
        ${CMAKE_CURRENT_LIST_DIR}/timer.c
)

set(
        CCRF_PLATFORM_LIBS
        kio
        fw_board
)
