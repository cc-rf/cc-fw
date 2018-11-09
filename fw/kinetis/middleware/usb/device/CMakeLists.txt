if(CONFIG_KINETIS_MIDDLEWARE_USB_DEVICE)

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/usb_device_dci.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_device_ehci.c
            ${CMAKE_CURRENT_LIST_DIR}/usb_device_khci.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
    )

endif()
