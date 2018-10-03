set(NAME fw_project_cloudchaser)

add_executable(
        ${NAME}
        main.c
        led.c
        lp5562.c
        cloudchaser.c
        console.c
        rf_uart.c
        usb/composite.c
        usb/usb_device_cdc_acm.c
        usb/usb_device_ch9.c
        usb/usb_device_class.c
        usb/usb_device_descriptor.c
        usb/virtual_com.c
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
        PUBLIC kinetis_middleware_usb
        PUBLIC kinetis_middleware_usb_phy
        PUBLIC kinetis_middleware_usb_device
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

add_library(kinetis_middleware_usb_config INTERFACE)
target_include_directories(kinetis_middleware_usb_config INTERFACE usb/include)
