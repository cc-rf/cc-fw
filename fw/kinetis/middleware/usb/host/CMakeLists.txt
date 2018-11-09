if(CONFIG_KINETIS_MIDDLEWARE_USB_HOST)

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/usb_host_devices.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_host_ehci.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_host_framework.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_host_hci.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_host_khci.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_audio.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_cdc.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_hid.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_hub.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_hub_app.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_msd.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_msd_ufi.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_phdc.c
            ${CMAKE_CURRENT_LIST_DIR}/class/usb_host_printer.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/class/include
    )

endif()
