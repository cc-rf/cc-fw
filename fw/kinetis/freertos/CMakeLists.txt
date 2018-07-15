set(NAME kinetis_freertos)

set(FREERTOS_KINETIS_DRIVERS)

if(FREERTOS_KINETIS_DRIVER_DSPI)
    list(APPEND FREERTOS_KINETIS_DRIVERS driver/fsl_dspi_freertos.c)
endif()

if(FREERTOS_KINETIS_DRIVER_I2C)
    list(APPEND FREERTOS_KINETIS_DRIVERS driver/fsl_i2c_freertos.c)
endif()

if(FREERTOS_KINETIS_DRIVER_LPUART)
    list(APPEND FREERTOS_KINETIS_DRIVERS driver/fsl_lpuart_freertos.c)
endif()

if(FREERTOS_KINETIS_DRIVER_UART)
    list(APPEND FREERTOS_KINETIS_DRIVERS driver/fsl_uart_freertos.c)
endif()

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        croutine.c
        event_groups.c
        list.c
        queue.c
        tasks.c
        timers.c
        portable/GCC/ARM_CM4F/fsl_tickless_lptmr.c
        portable/GCC/ARM_CM4F/port.c
        portable/MemMang/heap_4.c
        ${FREERTOS_KINETIS_DRIVERS}
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC portable/GCC/ARM_CM4F/include
        PUBLIC driver/include
)

target_link_libraries(
        ${NAME}
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos_config
        PUBLIC kio
)

target_compile_definitions(
        ${NAME}
        PUBLIC FREERTOS
)

add_library(freertos ALIAS kinetis_freertos)
