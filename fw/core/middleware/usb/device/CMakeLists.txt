set(NAME fw_core_middleware_usb_device)

add_library(
        ${NAME} STATIC
        usb_device_dci.c
        usb_device_ehci.c
        usb_device_khci.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_middleware_usb
        PRIVATE fw_core_cpu
)
