target_sources(
        ${TARGET} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/board.c
        ${CMAKE_CURRENT_LIST_DIR}/clock.c
        ${CMAKE_CURRENT_LIST_DIR}/pins.c
        ${CMAKE_CURRENT_LIST_DIR}/led.c
        ${CMAKE_CURRENT_LIST_DIR}/lp5562.c
)

target_include_directories(
        ${TARGET}
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(
        ${TARGET}
        PUBLIC BOARD_CLOUDCHASER=1
        PUBLIC BOARD_REVISION=${CONFIG_FW_BOARD_REV}
)
