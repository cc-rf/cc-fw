set(NAME kinetis_middleware_usb)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)

target_link_libraries(
        ${NAME}
        INTERFACE kinetis_middleware_usb_osa
        INTERFACE kinetis_sdk_driver
        INTERFACE kinetis_middleware_usb_config
)

add_subdirectory(device)
add_subdirectory(host)
add_subdirectory(osa)
add_subdirectory(phy)
