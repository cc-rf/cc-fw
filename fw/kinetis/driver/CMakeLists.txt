set(NAME kinetis_sdk_driver)

set(KINETIS_SDK_DRIVERS)

list(APPEND KINETIS_SDK_DRIVERS fsl_common.c)

if(KINETIS_SDK_DRIVER_ADC16)
    list(APPEND KINETIS_SDK_DRIVERS fsl_adc16.c)
endif()

if(KINETIS_SDK_DRIVER_CMP)
    list(APPEND KINETIS_SDK_DRIVERS fsl_cmp.c)
endif()

if(KINETIS_SDK_DRIVER_CRC)
    list(APPEND KINETIS_SDK_DRIVERS fsl_crc.c)
endif()

if(KINETIS_SDK_DRIVER_DAC)
    list(APPEND KINETIS_SDK_DRIVERS fsl_dac.c)
endif()

if(KINETIS_SDK_DRIVER_DMAMUX)
    list(APPEND KINETIS_SDK_DRIVERS fsl_dmamux.c)
endif()

if(KINETIS_SDK_DRIVER_DSPI_EDMA)
    set(KINETIS_SDK_DRIVER_DSPI TRUE)
    set(KINETIS_SDK_DRIVER_EDMA TRUE)
    list(APPEND KINETIS_SDK_DRIVERS fsl_dspi_edma.c)
endif()

if(KINETIS_SDK_DRIVER_DSPI)
    list(APPEND KINETIS_SDK_DRIVERS fsl_dspi.c)
endif()

if(KINETIS_SDK_DRIVER_I2C_EDMA)
    set(KINETIS_SDK_DRIVER_I2C TRUE)
    set(KINETIS_SDK_DRIVER_EDMA TRUE)
    list(APPEND KINETIS_SDK_DRIVERS fsl_i2c_edma.c)
endif()

if(KINETIS_SDK_DRIVER_I2C)
    list(APPEND KINETIS_SDK_DRIVERS fsl_i2c.c)
endif()

if(KINETIS_SDK_DRIVER_LPUART_EDMA)
    set(KINETIS_SDK_DRIVER_LPUART TRUE)
    set(KINETIS_SDK_DRIVER_EDMA TRUE)
    list(APPEND KINETIS_SDK_DRIVERS fsl_lpuart_edma.c)
endif()

if(KINETIS_SDK_DRIVER_LPUART)
    list(APPEND KINETIS_SDK_DRIVERS fsl_lpuart.c)
endif()

if(KINETIS_SDK_DRIVER_SAI_EDMA)
    set(KINETIS_SDK_DRIVER_SAI TRUE)
    set(KINETIS_SDK_DRIVER_EDMA TRUE)
    list(APPEND KINETIS_SDK_DRIVERS fsl_sai_edma.c)
endif()

if(KINETIS_SDK_DRIVER_SAI)
    list(APPEND KINETIS_SDK_DRIVERS fsl_sai.c)
endif()

if(KINETIS_SDK_DRIVER_UART_EDMA)
    set(KINETIS_SDK_DRIVER_UART TRUE)
    set(KINETIS_SDK_DRIVER_EDMA TRUE)
    list(APPEND KINETIS_SDK_DRIVERS fsl_uart_edma.c)
endif()

if(KINETIS_SDK_DRIVER_UART)
    list(APPEND KINETIS_SDK_DRIVERS fsl_uart.c)
endif()

if(KINETIS_SDK_DRIVER_EDMA)
    list(APPEND KINETIS_SDK_DRIVERS fsl_edma.c)
endif()

if(KINETIS_SDK_DRIVER_EWM)
    list(APPEND KINETIS_SDK_DRIVERS fsl_ewm.c)
endif()

if(KINETIS_SDK_DRIVER_FLASH)
    list(APPEND KINETIS_SDK_DRIVERS fsl_ftfx_flash.c fsl_ftfx_controller.c)
endif()

if(KINETIS_SDK_DRIVER_FLEXBUS)
    list(APPEND KINETIS_SDK_DRIVERS fsl_flexbus.c)
endif()

if(KINETIS_SDK_DRIVER_FTM)
    list(APPEND KINETIS_SDK_DRIVERS fsl_ftm.c)
endif()

if(KINETIS_SDK_DRIVER_GPIO)
    list(APPEND KINETIS_SDK_DRIVERS fsl_gpio.c)
endif()

if(KINETIS_SDK_DRIVER_LLWU)
    list(APPEND KINETIS_SDK_DRIVERS fsl_llwu.c)
endif()

if(KINETIS_SDK_DRIVER_LPTMR)
    list(APPEND KINETIS_SDK_DRIVERS fsl_lptmr.c)
endif()

if(KINETIS_SDK_DRIVER_PDB)
    list(APPEND KINETIS_SDK_DRIVERS fsl_pdb.c)
endif()

if(KINETIS_SDK_DRIVER_PIT)
    list(APPEND KINETIS_SDK_DRIVERS fsl_pit.c)
endif()

if(KINETIS_SDK_DRIVER_PMC)
    list(APPEND KINETIS_SDK_DRIVERS fsl_pmc.c)
endif()

if(KINETIS_SDK_DRIVER_RCM)
    list(APPEND KINETIS_SDK_DRIVERS fsl_rcm.c)
endif()

if(KINETIS_SDK_DRIVER_RNGA)
    list(APPEND KINETIS_SDK_DRIVERS fsl_rnga.c)
endif()

if(KINETIS_SDK_DRIVER_RTC)
    list(APPEND KINETIS_SDK_DRIVERS fsl_rtc.c)
endif()

if(KINETIS_SDK_DRIVER_SIM)
    list(APPEND KINETIS_SDK_DRIVERS fsl_sim.c)
endif()

if(KINETIS_SDK_DRIVER_SMC)
    list(APPEND KINETIS_SDK_DRIVERS fsl_smc.c)
endif()

if(KINETIS_SDK_DRIVER_VREF)
    list(APPEND KINETIS_SDK_DRIVERS fsl_vref.c)
endif()

if(KINETIS_SDK_DRIVER_WDOG)
    list(APPEND KINETIS_SDK_DRIVERS fsl_wdog.c)
endif()


add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        ${KINETIS_SDK_DRIVERS}
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PUBLIC ENABLE_RAM_VECTOR_TABLE
)

if(KINETIS_SDK_DRIVER_FLASH)
    target_compile_definitions(
            ${NAME}
            PUBLIC FTFx_DRIVER_IS_FLASH_RESIDENT=0
    )

    target_link_libraries(
            ${NAME}
            PUBLIC fw_board # for trace/assert
            PUBLIC usr # needed for __fast_code
    )
endif()

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_cpu_driver
)
