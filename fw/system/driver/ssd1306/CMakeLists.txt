set(NAME ssd1306)

if(NOT DEFINED CONFIG_SSD1306_PLATFORM)
    set(CONFIG_SSD1306_PLATFORM "kinetis")
elseif(NOT CONFIG_SSD1306_PLATFORM STREQUAL "kinetis")
    message(FATAL_ERROR "Unknown CONFIG_SSD1306_PLATFORM ${CONFIG_SSD1306_PLATFORM}")
endif()

add_library(
        ${NAME} STATIC
        src/ssd1306.c
        src/ssd1306_font.c
)

target_include_directories(
        ${NAME}
        PUBLIC include
        PRIVATE src/include
)

target_link_libraries(
        ${NAME}
        PUBLIC usr
        PUBLIC ssd1306_${CONFIG_SSD1306_PLATFORM}
)

add_subdirectory(src/sys/${CONFIG_SSD1306_PLATFORM})