set(NAME kinetis_sdk_mk22f51212)

add_library(
        ${NAME} STATIC
        startup/startup_MK22F51212.S
        startup/system_MK22F51212.c
)

target_include_directories(
        ${NAME}
        PUBLIC cmsis
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cmsis
        "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/MK22FN512xxx12_flash.ld"
)

target_compile_definitions(
        ${NAME}
        PUBLIC CPU_MK22FN512VLH12 __STARTUP_CLEAR_BSS
)

add_subdirectory(driver)