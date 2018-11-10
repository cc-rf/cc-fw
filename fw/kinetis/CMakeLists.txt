add_subdirectory(cmsis)
add_subdirectory(cpu)
add_subdirectory(driver)
add_subdirectory(middleware)

add_library(kinetis_sdk ALIAS kinetis_sdk_${CONFIG_KINETIS_CPU})
