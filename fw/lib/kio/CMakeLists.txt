if(CONFIG_LIB_KIO)

    if(NOT DEFINED CONFIG_KIO_UART_COUNT)
        set(CONFIG_KIO_UART_COUNT 1 CACHE STRING "Number of UARTs that can be managed (-1: MAX, 0: INFINITE/ON-DEMAND)")
    endif()

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/flsh.c
            ${CMAKE_CURRENT_LIST_DIR}/iic.c
            ${CMAKE_CURRENT_LIST_DIR}/pit.c
            ${CMAKE_CURRENT_LIST_DIR}/sclk.c
            ${CMAKE_CURRENT_LIST_DIR}/uart.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/composite.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/usb_device_cdc_acm.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/usb_device_ch9.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/usb_device_class.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/usb_device_descriptor.c
            ${CMAKE_CURRENT_LIST_DIR}/usb/virtual_com.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/usb/include
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
    )

    target_compile_definitions(
            ${TARGET}
            PUBLIC CONFIG_KIO_UART_COUNT=${CONFIG_KIO_UART_COUNT}
    )

endif()
