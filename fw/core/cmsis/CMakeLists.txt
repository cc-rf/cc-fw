set(NAME fw_core_cmsis)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)

target_link_libraries(
        ${NAME}
        INTERFACE fw_core_cpu
)
