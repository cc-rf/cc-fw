set(NAME util_trace)

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
        #INTERFACE kinetis_freertos # TODO: Find a way to make things like this available automatically when requested
        INTERFACE kinetis_middleware_dma_manager
)
