set(NAME ccrf_kinetis)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        amp.c
        isrd.c
        kinetis.c
        spi.c
)

#target_include_directories(
#        ${NAME}
#)

target_link_libraries(
        ${NAME}
        PUBLIC ccrf
        PUBLIC kio
)
