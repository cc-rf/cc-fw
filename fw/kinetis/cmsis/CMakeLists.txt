set(NAME kinetis_sdk_cmsis)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)

target_link_libraries(
        ${NAME}
        INTERFACE kinetis_sdk_cpu
)
