if(CONFIG_LIB_USR)

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/serf.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
    )

endif()
