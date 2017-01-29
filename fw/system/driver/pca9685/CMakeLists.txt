set(NAME pca9685)

if(NOT DEFINED CONFIG_PCA9685_PLATFORM)
    set(CONFIG_PCA9685_PLATFORM "kinetis")
elseif(NOT CONFIG_PCA9685_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CONFIG_PCA9685_PLATFORM ${CONFIG_PCA9685_PLATFORM}")
endif()

add_library(
        ${NAME} STATIC
        src/pca9685.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PRIVATE src/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC pca9685_${CONFIG_PCA9685_PLATFORM}
)

add_subdirectory(src/sys/${CONFIG_PCA9685_PLATFORM})
