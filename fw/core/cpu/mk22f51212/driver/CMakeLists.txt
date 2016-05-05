set(NAME fw_core_cpu_mk22f51212_driver)

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
        PUBLIC fw_core_driver
)
