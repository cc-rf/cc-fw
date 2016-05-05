set(NAME fw_core_middleware_usb)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)

target_link_libraries(
        ${NAME}
        INTERFACE ${FW_USB_OSA}
        #INTERFACE fw_core_util
        INTERFACE fw_project
)

add_subdirectory(device)
add_subdirectory(host)
add_subdirectory(osa)
add_subdirectory(phy)
