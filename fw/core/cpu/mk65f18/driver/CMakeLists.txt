set(NAME fw_core_cpu_mk65f18_driver)

add_library(
        ${NAME} STATIC
        fsl_clock.c
        fsl_cmt.c
        fsl_enet.c
        fsl_flexcan.c
        fsl_lmem_cache.c
        fsl_mpu.c
        fsl_sdhc.c
        fsl_sdramc.c
        fsl_tpm.c
        fsl_tsi_v4.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_core_driver
)

#set_target_properties(fw_core_cpu_mk65f18_driver PROPERTIES
#        DRIVERS_AVAILABLE fsl_clock
#)