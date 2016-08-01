set(NAME ssd1306_kinetis)

add_library(
        ${NAME} STATIC
        ssd1306_io.c
        spi.c
)

target_link_libraries(
        ${NAME}
        PUBLIC ssd1306
        PUBLIC kinetis_freertos
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC fw_project           # TODO: Figure out a better way to pull in compile defs/flags
)
