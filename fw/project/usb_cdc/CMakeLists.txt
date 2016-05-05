set(NAME fw_project_usb_cdc)

add_library(
        ${NAME} STATIC
        usb_device_cdc_acm.c
        usb_device_ch9.c
        usb_device_class.c
        usb_device_descriptor.c
        virtual_com.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_board
        PUBLIC fw_core_middleware_usb_osa_bm
        PUBLIC fw_core_middleware_usb_phy
        PUBLIC fw_core_middleware_usb_device
)
