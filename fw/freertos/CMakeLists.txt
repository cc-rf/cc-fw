set(NAME kinetis_freertos)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        # croutine.c
        # event_groups.c
        list.c
        queue.c
        tasks.c
        timers.c
        # stream_buffer.c
        portable/GCC/ARM_CM4F/fsl_tickless_systick.c
        portable/GCC/ARM_CM4F/port.c
        #portable/MemMang/heap_4.c
        portable/MemMang/umm.c
        portable/MemMang/umm/umm_malloc.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PUBLIC portable/GCC/ARM_CM4F/include
        PUBLIC driver/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos_config
)

target_compile_definitions(
        ${NAME}
        PUBLIC FREERTOS
)

add_library(freertos ALIAS kinetis_freertos)
