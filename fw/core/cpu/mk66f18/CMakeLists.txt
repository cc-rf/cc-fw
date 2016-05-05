set(NAME fw_core_cpu_mk66f18)

if(NOT DEFINED FW_CPU_PART)
    set(FW_CPU_PART "MK66FN2M0VMD18")
endif()

if(FW_CPU_PART STREQUAL "MK66FN2M0VMD18")
    set(LDFILE_PREFIX "MK66FN2M0xxx18")
elseif(FW_CPU_PART STREQUAL "MK66FX1M0VMD18")
    set(LDFILE_PREFIX "MK66FX1M0xxx18")
else()
    message(FATAL_ERROR "Unknown FW_CPU_PART ${FW_CPU_PART}")
endif()

add_library(
        ${NAME} STATIC
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
        PUBLIC fw_core_cmsis
        "-Wl,-T${CMAKE_CURRENT_LIST_DIR}/${LDFILE_PREFIX}_flash.ld"
)

target_compile_definitions(
        ${NAME}
        PUBLIC -DCPU_${FW_CPU_PART} -D__STARTUP_CLEAR_BSS
)

add_subdirectory(driver)