set(NAME fw_board_twr_k65f180m)

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
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
)

target_compile_definitions(
        ${NAME}
        PUBLIC BOARD_TWR_K65F180M=1
)
