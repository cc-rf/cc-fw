set(NAME libcc_kinetis)

add_library(
        ${NAME} STATIC
        amp.c
        isrd.c
        kinetis.c
        pit.c
        spi.c
)

#target_include_directories(
#        ${NAME}
#)

target_link_libraries(
        ${NAME}
        PUBLIC libcc
        PUBLIC kinetis_freertos
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC fw_project           # TODO: Figure out a better way to pull in compile defs/flags
)
