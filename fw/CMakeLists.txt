cmake_minimum_required(VERSION 3.4)
project(fw C ASM)
set(FW_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include(armgcc)

set(CFLAGS_STD "-std=gnu11 -Wall -D_GNU_SOURCE -fstack-usage -Wstack-usage=512")
set(CFLAGS_STD_DEBUG "-O0 -g3 -D_DEBUG=1 -DDEBUG")
set(CFLAGS_STD_RELEASE "-O3 -g0 -D_DEBUG=0")

set(CFLAGS_KINETIS "-fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" )
set(CFLAGS_CPU_CM4F "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" )

# rdimon is for SWO/Semihosting, nosys is for without
# NOTE: what about rdpmon? and redboot?
#set(SPECS "-specs=rdimon.specs -specs=nano.specs")
set(SPECS "-specs=nosys.specs -specs=nano.specs")
set(LFLAGS_FW "-Wl,--gc-sections -Wl,-Map,\"${PROJECT_NAME}.map\" -Wl,--no-wchar-size-warning ${SPECS}")

set(CFLAGS_BASE "${CFLAGS_STD} ${CFLAGS_KINETIS} ${CFLAGS_CPU_CM4F}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CFLAGS_BASE}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS_BASE}" CACHE INTERNAL "CXX Flags")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE INTERNAL "CXX Flags (DEBUG)")
set(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE INTERNAL "ASM Flags (DEBUG)")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "C Flags (RELEASE)")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "CXX Flags (RELEASE)")
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "ASM Flags (RELEASE)")

set(CMAKE_EXE_LINKER_FLAGS "${LFLAGS_FW}")

if(NOT DEFINED CONFIG_FW_PROJECT)
    message(FATAL_ERROR "CONFIG_FW_PROJECT is required")
endif()

set(FW_PROJECT_TARGET fw_project_${CONFIG_FW_PROJECT})
set(FW_PROJECT_PATH "project/${CONFIG_FW_PROJECT}")

set(RUNTIME_OUTPUT_DIRECTORY "${BUILD_ROOT}")
set(EXECUTABLE_OUTPUT_PATH "${RUNTIME_OUTPUT_DIRECTORY}")
#set(LIBRARY_OUTPUT_DIRECTORY "${BUILD_ROOT}/lib")
#set(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_DIRECTORY}")


#TODO: Check that TARGET is set? or maybe keep as "firmware" always

include("${FW_PROJECT_PATH}/config.cmake")

if(DEFINED CONFIG_FW_BOARD)
    set(FW_BOARD_PATH "board/${CONFIG_FW_BOARD}")
    include("${FW_BOARD_PATH}/config.cmake")
endif()

add_subdirectory(board)
add_subdirectory(freertos)
add_subdirectory(kinetis)
add_subdirectory(lib)
add_subdirectory(project)

set_target_properties(
        ${FW_PROJECT_TARGET} PROPERTIES
        OUTPUT_NAME "fw"
)

set(TARGET "${PROJECT_NAME}")
add_executable(${TARGET} ALIAS ${FW_PROJECT_TARGET})

add_custom_target(
        fw_hex
        COMMAND ${CMAKE_OBJCOPY} -O ihex -S "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.hex"
        COMMAND size -A -d "${PROJECT_NAME}.hex"
        BYPRODUCTS "${PROJECT_NAME}.hex"
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

add_custom_target(
        fw_bin
        COMMAND ${CMAKE_OBJCOPY} -O binary -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin"
        COMMAND size -A -d "${PROJECT_NAME}.bin"
        BYPRODUCTS "${PROJECT_NAME}.bin"
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

add_custom_target(
        fw_srec
        COMMAND ${CMAKE_OBJCOPY} -O srec -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.srec"
        COMMAND size -A -d "${PROJECT_NAME}.srec"
        BYPRODUCTS "${PROJECT_NAME}.srec"
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

# https://github.com/energia/Energia/wiki/FLASH-and-estimated-RAM-Usage
# text + data = flash
# bss  + data = ram
add_custom_target(
        fw_siz
        COMMAND ${CMAKE_OBJSIZE} --format=berkeley --totals "$<TARGET_FILE:${TARGET}>"
        COMMAND size -A -d "$<TARGET_FILE:${TARGET}>"
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

# add_dependencies(fw_hex ${TARGET})
