set(NAME kinetis_sdk_mk22f51212_driver)

add_library(
        ${NAME} STATIC
        fsl_clock.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cpu
        PUBLIC kinetis_sdk_driver
)
