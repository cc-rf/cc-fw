set(NAME fw_core_freertos)

add_library(
        ${NAME} STATIC
        croutine.c
        event_groups.c
        list.c
        queue.c
        tasks.c
        timers.c
        portable/GCC/ARM_CM4F/port.c
        portable/MemMang/heap_1.c
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
        PUBLIC fw_core_driver
        PUBLIC fw_project # TODO: replace with whatever the project is or don't do this just to get FreeRTOSConfig.h
        # idea: have a target somewhere called fw_core_freertos_config that is aliased to a per-project thing
)

target_compile_definitions(
        ${NAME}
        PUBLIC -DFREERTOS
)