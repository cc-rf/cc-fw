if(CONFIG_KINETIS_MIDDLEWARE_DMA_MANAGER)

    target_sources(
            ${TARGET} PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/fsl_dma_manager.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
    )

endif()
