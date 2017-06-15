set(NAME mcuc)

if(NOT DEFINED CONFIG_MCUC_PLATFORM)
    set(CONFIG_MCUC_PLATFORM "kinetis")
elseif(NOT CONFIG_MCUC_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CONFIG_MCUC_PLATFORM ${CONFIG_MCUC_PLATFORM}")
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        src/${CONFIG_MCUC_PLATFORM}/master.c
        src/${CONFIG_MCUC_PLATFORM}/slave.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PRIVATE kinetis_sdk
        PRIVATE kinetis_sdk_driver
        PRIVATE kinetis_freertos
        PRIVATE kinetis_middleware_dma_manager
        PRIVATE fw_system_framework_trace
)
