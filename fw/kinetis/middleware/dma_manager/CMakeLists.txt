set(NAME kinetis_middleware_dma_manager)

add_library(
        ${NAME} STATIC
        fsl_dma_manager.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_driver
)
