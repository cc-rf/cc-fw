set(NAME fw_project_usb_hid_generic_freertos)

add_library(
        ${NAME} STATIC
        hid_generic.c
        usb_device_ch9.c
        usb_device_class.c
        usb_device_descriptor.c
        usb_device_hid.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_board
        PUBLIC fw_core_freertos
        PUBLIC fw_core_middleware_usb_phy
        PUBLIC fw_core_middleware_usb_device
)
