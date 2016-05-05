set(NAME fw_core_board_cloudchaser)

add_library(
        ${NAME} STATIC
        board.c
        clock_config.c
        pin_mux.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_cpu_mk66f18
        PUBLIC fw_core_driver
)

target_compile_definitions(
        ${NAME}
        PUBLIC -DBOARD_CLOUDCHASER=1
)
