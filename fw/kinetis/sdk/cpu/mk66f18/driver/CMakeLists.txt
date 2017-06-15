set(NAME kinetis_sdk_mk66f18_driver)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        fsl_clock.c
        fsl_cmt.c
        fsl_enet.c
        fsl_flexcan.c
        fsl_lmem_cache.c
        fsl_sdhc.c
        fsl_sdramc.c
        fsl_sysmpu.c
        fsl_tpm.c
        fsl_tsi_v4.c
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
