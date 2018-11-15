set(NAME fw_project_cloudchaser)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        cloudchaser.c
        # console.c
        rf_uart.c
        ccio.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC usb/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC fw_board
        PUBLIC kio
        PUBLIC ccrf
)

add_library(kinetis_freertos_config INTERFACE)
target_include_directories(kinetis_freertos_config INTERFACE include)

