set(NAME pca9685_kinetis)

add_library(
        ${NAME} STATIC
        i2c.c
        pca9685_io.c
)

target_include_directories(
        ${NAME}
        PRIVATE ../../include
)

target_link_libraries(
        ${NAME}
        PUBLIC pca9685
        PUBLIC kinetis_freertos
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_middleware_dma_manager
        PUBLIC fw_project           # TODO: Figure out a better way to pull in compile defs/flags
)
