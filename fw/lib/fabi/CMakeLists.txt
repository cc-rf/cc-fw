set(NAME fabi) # Fast Blinky Interface

if(CONFIG_FABI)

add_library(
        ${NAME} STATIC EXCLUDE_FROM_ALL
        fabi.c include/fabi.h
        ftm.c ftm.h
)

target_include_directories(
        ${NAME}
        PUBLIC include
)

target_compile_definitions(
        ${NAME}
        PUBLIC FABI=1
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC kio
        PUBLIC kinetis_sdk
        PUBLIC kinetis_sdk_driver
        PUBLIC kinetis_freertos
        PUBLIC kinetis_middleware_dma_manager
)

endif()
