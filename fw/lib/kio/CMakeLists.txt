set(NAME kio)

if(NOT DEFINED CONFIG_KIO_UART_COUNT)
    set(CONFIG_KIO_UART_COUNT 1 CACHE STRING "Number of UARTs that can be managed (-1: MAX, 0: INFINITE/ON-DEMAND)")
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        iic.c
        pit.c
        sclk.c
        uart.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PRIVATE CONFIG_KIO_UART_COUNT=${CONFIG_KIO_UART_COUNT}
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_dma_manager
)
