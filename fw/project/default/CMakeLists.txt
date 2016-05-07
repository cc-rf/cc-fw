set(NAME fw_project_default)

add_library(
        ${NAME} STATIC
        main.c
        libcc/src/io.c
        libcc/src/interface/kinetis/amp.c
        libcc/src/interface/kinetis/kinetis.c
        libcc/src/interface/kinetis/spi.c
        libcc/src/interface/kinetis/isr.c
        libcc/src/interface/kinetis/pit.c
        libcc/src/chan.c
        libcc/src/freq.c
        libcc/src/phy.c
        libcc/src/mac.c
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
        PUBLIC libcc/include
        #PRIVATE
        libcc/src/include
        usb/include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_board
        PUBLIC fw_core_freertos
        PUBLIC fw_core_middleware_usb
        PUBLIC fw_core_middleware_usb_phy
        PUBLIC fw_core_middleware_usb_device
        PUBLIC fw_core_middleware_dma_manager
        PUBLIC fw_system_framework_trace
)
