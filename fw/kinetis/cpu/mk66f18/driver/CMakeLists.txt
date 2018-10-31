set(NAME kinetis_sdk_mk66f18_driver)

set(KINETIS_SDK_CPU_DRIVERS)

list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_clock.c)

if(KINETIS_SDK_DRIVER_CMT)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_cmt.c)
endif()

if(KINETIS_SDK_DRIVER_ENET)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_enet.c)
endif()

if(KINETIS_SDK_DRIVER_FLEXCAN)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_flexcan.c)
endif()

if(KINETIS_SDK_DRIVER_LMEM_CACHE)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_lmem_cache.c)
endif()

if(KINETIS_SDK_DRIVER_SDHC)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_sdhc.c)
endif()

if(KINETIS_SDK_DRIVER_SDRAMC)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_sdramc.c)
endif()

if(KINETIS_SDK_DRIVER_SYSMPU)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_sysmpu.c)
endif()

if(KINETIS_SDK_DRIVER_TPM)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_tpm.c)
endif()

if(KINETIS_SDK_DRIVER_TSI)
    list(APPEND KINETIS_SDK_CPU_DRIVERS fsl_tsi_v4.c)
endif()


add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        ${KINETIS_SDK_CPU_DRIVERS}
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cpu
        PUBLIC kinetis_sdk_driver
)
