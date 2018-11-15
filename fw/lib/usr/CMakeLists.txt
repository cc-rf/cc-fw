set(NAME usr)

set(CONFIG_MBUF_TRACE 0 CACHE INTEGER "Enable mbuf trace output")
set(CONFIG_MBUF_VALIDATE 0 CACHE INTEGER "Rigorously validate all input mbufs")

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

target_compile_definitions(
        ${NAME}
        PUBLIC CONFIG_MBUF_TRACE=${CONFIG_MBUF_TRACE}
        PUBLIC CONFIG_MBUF_VALIDATE=${CONFIG_MBUF_VALIDATE}
)

target_link_libraries(
        ${NAME}
        PUBLIC freertos
)
