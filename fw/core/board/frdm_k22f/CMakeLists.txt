set(NAME fw_core_board_frdm_k22f)

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
        PUBLIC fw_core_cpu_mk22f51212
        PUBLIC fw_core_driver
)

target_compile_definitions(
        ${NAME}
        PUBLIC -DBOARD_FRDM_K22F=1
)
