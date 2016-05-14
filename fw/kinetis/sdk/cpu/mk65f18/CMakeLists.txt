set(NAME kinetis_sdk_mk65f18)

add_library(
        ${NAME} STATIC
        startup/startup_MK65F18.S
        startup/system_MK65F18.c
)

target_include_directories(
        ${NAME}
        PUBLIC cmsis
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cmsis
        "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/MK65FN2M0xxx18_flash.ld"
)

target_compile_definitions(
        ${NAME}
        PUBLIC CPU_MK65FN2M0VMI18 __STARTUP_CLEAR_BSS
)

add_subdirectory(driver)