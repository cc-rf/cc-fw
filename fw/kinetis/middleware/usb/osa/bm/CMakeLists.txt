target_sources(
        ${TARGET} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/usb_osa_bm.c
)

target_include_directories(
        ${TARGET}
        PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
)

target_compile_definitions(
        ${TARGET}
        PUBLIC USB_STACK_BM
)
