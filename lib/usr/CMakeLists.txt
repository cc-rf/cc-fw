set(NAME usr)

add_library(
        ${NAME} INTERFACE
)

target_include_directories(
        ${NAME}
        INTERFACE include
)
