set(NAME fw_project_usb)

add_library(
        ${NAME} STATIC
        main.c
        usb/usb_device_cdc_acm.c
        usb/usb_device_ch9.c
        usb/usb_device_class.c
        usb/usb_device_descriptor.c
        usb/virtual_com.c
        serf.c
)

#set_source_files_properties(main.c PROPERTIES COMPILE_FLAGS "-Wa,-a,-ad")

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC usb/include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_board
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_usb
        PUBLIC kinetis_middleware_usb_phy
        PUBLIC kinetis_middleware_usb_device
        PUBLIC kinetis_middleware_dma_manager
        PUBLIC fw_system_framework_trace
        PUBLIC util_uart
        PUBLIC libcc
        PUBLIC mcuc
        "-Xlinker -defsym=__heap_size__=0x4000"
        "-Xlinker -defsym=__stack_size__=0x500"
)

add_library(kinetis_freertos_config INTERFACE)
target_include_directories(kinetis_freertos_config INTERFACE include)

add_library(kinetis_middleware_usb_config INTERFACE)
target_include_directories(kinetis_middleware_usb_config INTERFACE usb/include)
