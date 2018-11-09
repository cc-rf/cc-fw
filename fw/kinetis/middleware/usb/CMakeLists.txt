if(CONFIG_KINETIS_MIDDLEWARE_USB)

    target_include_directories(
        ${TARGET}
        PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include"
    )

    add_subdirectory(device)
    add_subdirectory(host)
    add_subdirectory(osa)
    add_subdirectory(phy)

endif()
