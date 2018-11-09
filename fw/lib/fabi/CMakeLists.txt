if(CONFIG_LIB_FABI)

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/fabi.c
            ${CMAKE_CURRENT_LIST_DIR}/ftm.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
    )

    target_compile_definitions(
            ${TARGET}
            PUBLIC FABI=1
    )

endif()
