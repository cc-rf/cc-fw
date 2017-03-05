set(NAME util_uart)

if(NOT DEFINED CONFIG_UTIL_UART_COUNT)
    set(CONFIG_UTIL_UART_COUNT 1 CACHE STRING "Number of UARTs that can be managed (-1: MAX, 0: INFINITE/ON-DEMAND)")
endif()

add_library(
        ${NAME} STATIC
        uart.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PRIVATE CONFIG_UTIL_UART_COUNT=${CONFIG_UTIL_UART_COUNT}
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_dma_manager
)
