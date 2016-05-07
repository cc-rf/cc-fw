set(NAME fw_core_middleware_dma_manager)

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
        PUBLIC fw_core_driver
)
