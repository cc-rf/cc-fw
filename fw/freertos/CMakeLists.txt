if(CONFIG_FREERTOS)

    target_sources(
            ${TARGET} PRIVATE
            # ${CMAKE_CURRENT_LIST_DIR}/croutine.c
            # ${CMAKE_CURRENT_LIST_DIR}/event_groups.c
            ${CMAKE_CURRENT_LIST_DIR}/list.c
            ${CMAKE_CURRENT_LIST_DIR}/queue.c
            ${CMAKE_CURRENT_LIST_DIR}/tasks.c
            ${CMAKE_CURRENT_LIST_DIR}/timers.c
            # ${CMAKE_CURRENT_LIST_DIR}/stream_buffer.c
            ${CMAKE_CURRENT_LIST_DIR}/portable/GCC/ARM_CM4F/fsl_tickless_systick.c
            ${CMAKE_CURRENT_LIST_DIR}/portable/GCC/ARM_CM4F/port.c
            ${CMAKE_CURRENT_LIST_DIR}/portable/MemMang/heap_4.c
    )

    target_include_directories(
            ${TARGET}
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/driver/include
            PRIVATE ${CMAKE_CURRENT_LIST_DIR}/portable/GCC/ARM_CM4F/include
    )

endif()
