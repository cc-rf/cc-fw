set(NAME kinetis_sdk_driver)

add_library(
        ${NAME} STATIC
        fsl_adc16.c
        fsl_cmp.c
        fsl_common.c
        fsl_crc.c
        fsl_dac.c
        fsl_dmamux.c
        fsl_dspi.c
        fsl_dspi_edma.c
        fsl_edma.c
        fsl_ewm.c
        fsl_flash.c
        fsl_flexbus.c
        fsl_ftm.c
        fsl_gpio.c
        fsl_i2c.c
        fsl_i2c_edma.c
        fsl_llwu.c
        fsl_lptmr.c
        fsl_lpuart.c
        fsl_lpuart_edma.c
        fsl_pdb.c
        fsl_pit.c
        fsl_pmc.c
        fsl_rcm.c
        fsl_rnga.c
        fsl_rtc.c
        fsl_sai.c
        fsl_sai_edma.c
        fsl_sim.c
        fsl_smc.c
        fsl_uart.c
        fsl_uart_edma.c
        fsl_vref.c
        fsl_wdog.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cpu_driver
)
