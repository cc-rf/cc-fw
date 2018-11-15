set(NAME usr)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        include/usr/cobs.h
        include/usr/mbuf.h
        include/usr/serf.h
        include/usr/type.h
        mbuf.c
        serf.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC freertos
)
