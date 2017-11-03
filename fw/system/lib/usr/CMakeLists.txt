set(NAME usr)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        serf.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PRIVATE freertos
)
