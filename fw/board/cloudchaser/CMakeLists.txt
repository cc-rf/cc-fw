set(NAME fw_board_cloudchaser)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        board.c
        clock.c
        pins.c
        led.c
        lp5562.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC kio
)

if(CONFIG_FABI)

    target_link_libraries(
            ${NAME}
            PUBLIC fabi
    )

endif()

target_compile_definitions(
        ${NAME}
        PUBLIC BOARD_CLOUDCHASER=1
        PUBLIC BOARD_REVISION=${CONFIG_FW_BOARD_REV}
        PUBLIC CONFIG_DISABLE_BOARD_TRACE=${CONFIG_DISABLE_BOARD_TRACE}
)
