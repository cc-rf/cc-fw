set(NAME kinetis_middleware_usb_host)

add_library(
        ${NAME} STATIC
        usb_host_devices.c
        usb_host_ehci.c
        usb_host_framework.c
        usb_host_hci.c
        usb_host_khci.c
        class/usb_host_audio.c
        class/usb_host_cdc.c
        class/usb_host_hid.c
        class/usb_host_hub.c
        class/usb_host_hub_app.c
        class/usb_host_msd.c
        class/usb_host_msd_ufi.c
        class/usb_host_phdc.c
        class/usb_host_printer.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC class/include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_middleware_usb
        PRIVATE kinetis_sdk
)
