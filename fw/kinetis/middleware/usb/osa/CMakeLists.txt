if(NOT DEFINED CONFIG_KINETIS_MIDDLEWARE_USB_OSA)
    set(CONFIG_KINETIS_MIDDLEWARE_USB_OSA "freertos"
            CACHE STRING "Kinetis USB Middleware OSA")
endif()

add_subdirectory(${CONFIG_KINETIS_MIDDLEWARE_USB_OSA})

add_library(kinetis_middleware_usb_osa ALIAS kinetis_middleware_usb_osa_${CONFIG_KINETIS_MIDDLEWARE_USB_OSA})