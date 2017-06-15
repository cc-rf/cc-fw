set(NAME kinetis_freertos)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        croutine.c
        event_groups.c
        list.c
        queue.c
        tasks.c
        timers.c
        portable/GCC/ARM_CM4F/fsl_tickless_lptmr.c
        portable/GCC/ARM_CM4F/fsl_tickless_systick.c
        portable/GCC/ARM_CM4F/port.c
        portable/MemMang/heap_3.c
        driver/fsl_i2c_freertos.c
        driver/fsl_dspi_freertos.c
        driver/fsl_lpuart_freertos.c
        driver/fsl_uart_freertos.c
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
)

target_compile_definitions(
        ${NAME}
        PUBLIC FREERTOS
)
