set(NAME fw_system_framework_trace)

add_library(
        ${NAME} STATIC
        freertos/uart.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_system_common
        PUBLIC fw_core_cmsis
        PUBLIC fw_project
)
