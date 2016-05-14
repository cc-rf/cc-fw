set(NAME kinetis_middleware_usb_osa_bm)

add_library(
        ${NAME} STATIC
        usb_osa_bm.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PUBLIC USB_STACK_BM
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_middleware_usb
        PRIVATE kinetis_sdk
)
