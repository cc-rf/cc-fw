set(NAME fw_core_middleware_usb_phy)

add_library(
        ${NAME} STATIC
        usb_phy.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PRIVATE fw_core_middleware_usb
        PRIVATE fw_core_cpu
)
