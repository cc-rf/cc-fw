set(NAME libcc)

if(NOT DEFINED CONFIG_LIBCC_PLATFORM)
    set(CONFIG_LIBCC_PLATFORM "kinetis")
elseif(NOT CONFIG_LIBCC_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CONFIG_LIBCC_PLATFORM ${CONFIG_LIBCC_PLATFORM}")
endif()

add_library(
        ${NAME} STATIC
        src/io.c
        src/chan.c
        src/freq.c
        src/phy.c
        src/mac.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC src/include
)

target_link_libraries(
        ${NAME}
        PUBLIC libcc_${CONFIG_LIBCC_PLATFORM}
)

add_subdirectory(src/sys/${CONFIG_LIBCC_PLATFORM})