set(NAME kio)

if(NOT DEFINED CONFIG_KIO_UART_COUNT)
    set(CONFIG_KIO_UART_COUNT 1 CACHE STRING "Number of UARTs that can be managed (-1: MAX, 0: INFINITE/ON-DEMAND)")
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        flsh.c
        iic.c
        pit.c
        sclk.c
        uart.c
        usb/composite.c
        usb/usb_device_cdc_acm.c
        usb/usb_device_ch9.c
        usb/usb_device_class.c
        usb/usb_device_descriptor.c
        usb/virtual_com.c
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
        PUBLIC fw_board
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_dma_manager
        PUBLIC kinetis_middleware_usb
        PUBLIC kinetis_middleware_usb_phy
        PUBLIC kinetis_middleware_usb_device
        PUBLIC kinetis_middleware_usb_config
)

add_library(kinetis_middleware_usb_config INTERFACE)
target_include_directories(kinetis_middleware_usb_config INTERFACE usb/include)

