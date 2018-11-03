set(NAME fw_project_cloudchaser)

add_executable(
        ${NAME}
        main.c
        cloudchaser.c
        console.c
        rf_uart.c
        ccio.c
)

#set_source_files_properties(main.c PROPERTIES COMPILE_FLAGS "-Wa,-a,-ad")

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC usb/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC fw_board
        PUBLIC kio
        PUBLIC ccrf
        PUBLIC fabi
)

target_compile_options(
        ${NAME}
        PUBLIC "-Wl,-defsym=__heap_size__=0x10000"
        PUBLIC "-Wl,-defsym=__stack_size__=0x500"
)

add_library(kinetis_freertos_config INTERFACE)
target_include_directories(kinetis_freertos_config INTERFACE include)

