set(NAME kinetis_middleware_usb_osa_freertos)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        usb_osa_freertos.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PUBLIC USB_STACK_FREERTOS
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_usb
        PRIVATE kinetis_sdk
)
