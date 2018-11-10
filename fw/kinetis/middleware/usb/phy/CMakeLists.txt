set(NAME kinetis_middleware_usb_phy)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        usb_phy.c
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
