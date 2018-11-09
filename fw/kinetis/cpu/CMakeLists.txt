if(NOT DEFINED CONFIG_KINETIS_CPU)
    set(CONFIG_KINETIS_CPU "mk66f18" CACHE STRING "Target Kinetis CPU")
endif()

add_subdirectory(${CONFIG_KINETIS_CPU})
