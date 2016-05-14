set(NAME fw_core_util)

add_library(
        ${NAME} STATIC
        fsl_debug_console.c
        fsl_notifier.c
        fsl_sbrk.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PRIVATE fw_core_driver m
)
