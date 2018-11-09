
target_sources(
        ${TARGET} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/cloudchaser.c
        ${CMAKE_CURRENT_LIST_DIR}/console.c
        ${CMAKE_CURRENT_LIST_DIR}/rf_uart.c
        ${CMAKE_CURRENT_LIST_DIR}/ccio.c
)

target_include_directories(
        ${TARGET}
        PRIVATE include
        PRIVATE usb/include
)
