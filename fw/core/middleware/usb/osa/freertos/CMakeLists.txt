set(NAME fw_core_middleware_usb_osa_freertos)

add_library(
        ${NAME} STATIC
        usb_osa_freertos.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PUBLIC -DUSB_STACK_FREERTOS
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_freertos
        PRIVATE fw_core_middleware_usb
        PRIVATE fw_core_cpu
)
