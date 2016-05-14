set(NAME fw_system_framework_trace)

add_library(
        ${NAME} STATIC
        freertos/uart.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_system_common
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_dma_manager
)
