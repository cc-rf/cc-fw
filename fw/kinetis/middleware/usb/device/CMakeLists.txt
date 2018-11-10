set(NAME kinetis_middleware_usb_device)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
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
        PUBLIC kinetis_middleware_usb
        PRIVATE kinetis_sdk
)
