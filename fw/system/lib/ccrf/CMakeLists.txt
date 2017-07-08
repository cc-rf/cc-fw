set(NAME ccrf)

if(NOT DEFINED CCRF_CONFIG_PLATFORM)
    set(CCRF_CONFIG_PLATFORM "kinetis")
elseif(NOT CCRF_CONFIG_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CCRF_CONFIG_PLATFORM ${CCRF_CONFIG_PLATFORM}")
endif()

include(sys/${CCRF_CONFIG_PLATFORM}/${CCRF_CONFIG_PLATFORM}.cmake)

set(
        CCRF_SOURCES
        net/mac.c
        phy/chan.c
        phy/phy.c
        rdio/rdio.c
        rdio/util.c
        ${CCRF_PLATFORM_SOURCES}
)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        ${CCRF_SOURCES}
)

target_compile_definitions(
        ${NAME} PUBLIC CCRF_CONFIG_PLATFORM_${CCRF_CONFIG_PLATFORM}=1
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PRIVATE ${CCRF_PLATFORM_LIBS}
)
