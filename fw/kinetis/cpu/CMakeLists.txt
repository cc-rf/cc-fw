if(NOT DEFINED CONFIG_KINETIS_CPU)
    set(CONFIG_KINETIS_CPU "mk66f18" CACHE STRING "Target Kinetis CPU")
endif()

add_subdirectory(${CONFIG_KINETIS_CPU})

add_library(kinetis_sdk_cpu ALIAS kinetis_sdk_${CONFIG_KINETIS_CPU})
add_library(kinetis_sdk_cpu_driver ALIAS kinetis_sdk_${CONFIG_KINETIS_CPU}_driver)