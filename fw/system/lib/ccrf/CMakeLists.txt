set(NAME ccrf)

if(NOT DEFINED CONFIG_CCRF_PLATFORM)
    set(CONFIG_CCRF_PLATFORM "kinetis")
elseif(NOT CONFIG_CCRF_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CONFIG_CCRF_PLATFORM ${CONFIG_CCRF_PLATFORM}")
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        src/io.c
        src/freq.c
        src/nphy.c
        src/nmac.c
        src/chan.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC src/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC ccrf_${CONFIG_CCRF_PLATFORM}
)

add_subdirectory(src/sys/${CONFIG_CCRF_PLATFORM})
