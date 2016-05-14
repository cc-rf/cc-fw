set(NAME fw_system_framework_trace)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)

target_link_libraries(
        ${NAME}
        INTERFACE usr
        INTERFACE kinetis_sdk
        INTERFACE kinetis_sdk_driver
        INTERFACE kinetis_freertos
        INTERFACE kinetis_middleware_dma_manager
)
