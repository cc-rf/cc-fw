set(NAME fw_core_middleware_usb_osa_bm)

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
        PUBLIC -DUSB_STACK_BM
)

target_link_libraries(
        ${NAME}
        PRIVATE fw_core_middleware_usb
        PRIVATE fw_core_cpu
)
