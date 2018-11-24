set(NAME kio)

if(NOT DEFINED CONFIG_KIO_UART_COUNT)
    set(CONFIG_KIO_UART_COUNT 1 CACHE STRING "Number of UARTs that can be managed (-1: MAX, 0: INFINITE/ON-DEMAND)")
endif()

add_custom_target(
        ${NAME}-date
        COMMAND echo -n "\\#define CONFIG_FLASH_TIMESTAMP   " > "${CMAKE_CURRENT_LIST_DIR}/info.h"
        COMMAND date "+%s" >> "${CMAKE_CURRENT_LIST_DIR}/info.h"
        COMMAND echo -n "\\#define CONFIG_FLASH_VERSION   0x" >> "${CMAKE_CURRENT_LIST_DIR}/info.h"
        COMMAND git log --oneline --max-count=1 --abbrev=8 | cut -d' ' -f 1 >> "${CMAKE_CURRENT_LIST_DIR}/info.h"
        COMMAND touch  "${CMAKE_CURRENT_LIST_DIR}/flsh.c"
        BYPRODUCTS "${CMAKE_CURRENT_LIST_DIR}/info.h"
)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        flsh.c
        iic.c
        itm.c
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

add_dependencies(${NAME} ${NAME}-date)

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
        PUBLIC ccrf # for FOTA
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
