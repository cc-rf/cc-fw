if(CONFIG_LIB_CCRF)

    if(NOT DEFINED CCRF_CONFIG_PLATFORM)
        set(CCRF_CONFIG_PLATFORM "kinetis")
    elseif(NOT CCRF_CONFIG_PLATFORM STREQUAL "kinetis")
        message(FATAL_ERROR "Unknown CCRF_CONFIG_PLATFORM ${CCRF_CONFIG_PLATFORM}")
    endif()

    include(sys/${CCRF_CONFIG_PLATFORM}/${CCRF_CONFIG_PLATFORM}.cmake)

    set(
            CCRF_SOURCES
            ${CMAKE_CURRENT_LIST_DIR}/include/ccrf/ccrf.h
            ${CMAKE_CURRENT_LIST_DIR}/include/ccrf/list.h
            ${CMAKE_CURRENT_LIST_DIR}/include/ccrf/mac.h
            ${CMAKE_CURRENT_LIST_DIR}/include/ccrf/net.h
            ${CMAKE_CURRENT_LIST_DIR}/include/ccrf/phy.h
            ${CMAKE_CURRENT_LIST_DIR}/net/mac.c
            ${CMAKE_CURRENT_LIST_DIR}/net/net.c
            ${CMAKE_CURRENT_LIST_DIR}/phy/chan.c
            ${CMAKE_CURRENT_LIST_DIR}/phy/phy.c
            ${CMAKE_CURRENT_LIST_DIR}/rdio/rdio.c
            ${CMAKE_CURRENT_LIST_DIR}/rdio/util.c
            ${CCRF_PLATFORM_SOURCES}
    )

    target_sources(
            ${TARGET} PRIVATE
            ${CCRF_SOURCES}
    )

    target_compile_definitions(
            ${TARGET} PUBLIC CCRF_CONFIG_PLATFORM_${CCRF_CONFIG_PLATFORM}=1
    )

    target_include_directories(
            ${TARGET}
            PRIVATE include
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}
    )

endif()
