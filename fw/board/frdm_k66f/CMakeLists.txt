set(NAME fw_board_frdm_k66f)

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
        PUBLIC BOARD_FRDM_K66F=1
)
