set(NAME fw_project_hello)

add_library(
        ${NAME} STATIC
        main.c
)

#set_source_files_properties(main.c PROPERTIES COMPILE_FLAGS "-Wa,-a,-ad")

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_link_libraries(
        ${NAME}
        PUBLIC fw_board
        PUBLIC kinetis_freertos
)
