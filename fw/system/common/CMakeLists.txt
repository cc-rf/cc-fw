set(NAME fw_system_common)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)
