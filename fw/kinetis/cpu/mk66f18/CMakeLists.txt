set(NAME kinetis_sdk_mk66f18)

if(NOT DEFINED CONFIG_KINETIS_CPU_PART)
    set(CONFIG_KINETIS_CPU_PART "MK66FN2M0VMD18" CACHE STRING "Target Kinetis CPU Part")
endif()

if(CONFIG_KINETIS_CPU_PART STREQUAL "MK66FN2M0VMD18")
    set(LDFILE_PREFIX "MK66FN2M0xxx18")
elseif(CONFIG_KINETIS_CPU_PART STREQUAL "MK66FX1M0VMD18")
    set(LDFILE_PREFIX "MK66FX1M0xxx18")
else()
    message(FATAL_ERROR "Unknown CONFIG_KINETIS_CPU_PART ${CONFIG_KINETIS_CPU_PART}")
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        startup/startup_MK66F18.S
        startup/system_MK66F18.c
)

target_include_directories(
        ${NAME}
        PUBLIC cmsis
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cmsis
        "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/${LDFILE_PREFIX}_flash.ld"
)

target_compile_definitions(
        ${NAME}
        PUBLIC CPU_${CONFIG_KINETIS_CPU_PART} __STARTUP_CLEAR_BSS
)

add_subdirectory(driver)
