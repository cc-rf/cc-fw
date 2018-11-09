target_sources(
        ${TARGET} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/usb_phy.c
)

target_include_directories(
        ${TARGET}
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)
