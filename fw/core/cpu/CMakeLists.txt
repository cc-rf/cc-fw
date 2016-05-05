add_subdirectory(mk22f51212)
add_subdirectory(mk65f18)
add_subdirectory(mk66f18)

if(TARGET fw_core_cpu_${FW_CPU_NAME})
    add_library(fw_core_cpu ALIAS fw_core_cpu_${FW_CPU_NAME})
endif()

if(TARGET fw_core_cpu_${FW_CPU_NAME}_driver)
    add_library(fw_core_cpu_driver ALIAS fw_core_cpu_${FW_CPU_NAME}_driver)
endif()