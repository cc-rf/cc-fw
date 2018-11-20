cmake_minimum_required(VERSION 3.4)
project(fw C ASM)
set(FW_ROOT "${CMAKE_CURRENT_LIST_DIR}")

set(CFLAGS_STD "-std=gnu11 -Wall -D_GNU_SOURCE -fstack-usage -Wstack-usage=512" CACHE STRING "" FORCE)
set(CFLAGS_STD_DEBUG "-O0 -g3 -D_DEBUG=1 -DDEBUG" CACHE STRING "" FORCE)
set(CFLAGS_STD_RELEASE "-Ofast -Wl,--gc-sections -g0 -D_DEBUG=0" CACHE STRING "" FORCE)

set(CFLAGS_KINETIS "-fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
set(CFLAGS_CPU_CM4F "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" CACHE STRING "" FORCE)

# rdimon is for SWO/Semihosting, nosys is for without
# NOTE: what about rdpmon? and redboot?
#set(SPECS "-specs=rdimon.specs -specs=nano.specs")
set(SPECS "-specs=nosys.specs -specs=nano.specs"  CACHE STRING "" FORCE)
set(LFLAGS_FW_EXE "-fwhole-program -Wl,-flto -Wl,--gc-sections -Wl,-Map,\"${PROJECT_NAME}.map\" -Wl,--no-wchar-size-warning ${SPECS}" CACHE STRING "" FORCE)

set(CFLAGS_BASE "${CFLAGS_STD} ${CFLAGS_KINETIS} ${CFLAGS_CPU_CM4F}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CFLAGS_BASE}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS_BASE}" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE STRING "" FORCE)

set(CMAKE_EXE_LINKER_FLAGS "${LFLAGS_FW_EXE}" CACHE STRING "" FORCE)

if(NOT DEFINED CONFIG_FW_PROJECT)
    message(FATAL_ERROR "CONFIG_FW_PROJECT is required")
endif()

set(FW_PROJECT_TARGET fw_project_${CONFIG_FW_PROJECT})
set(FW_PROJECT_PATH "project/${CONFIG_FW_PROJECT}")

set(RUNTIME_OUTPUT_DIRECTORY "${BUILD_ROOT}" CACHE STRING "" FORCE)
set(EXECUTABLE_OUTPUT_PATH "${RUNTIME_OUTPUT_DIRECTORY}" CACHE STRING "" FORCE)
#set(LIBRARY_OUTPUT_DIRECTORY "${BUILD_ROOT}/lib")
#set(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_DIRECTORY}")

include("${FW_PROJECT_PATH}/config.cmake")

if(DEFINED CONFIG_FW_BOARD)
    set(FW_BOARD_PATH "board/${CONFIG_FW_BOARD}")
    include("${FW_BOARD_PATH}/config.cmake")
endif()

# TODO: Read all config.cmake files recursively with glob?

add_subdirectory(board)
add_subdirectory(freertos)
add_subdirectory(kinetis)
add_subdirectory(lib)
add_subdirectory(project)


set(TARGET "${PROJECT_NAME}" CACHE STRING "Project Target")

message(STATUS "Project: ${FW_PROJECT_TARGET}")
message(STATUS "Target: ${TARGET}")

add_executable(
        ${TARGET}
        ${FW_PROJECT_PATH}/main.c
)

#set_source_files_properties(main.c PROPERTIES COMPILE_FLAGS "-Wa,-a,-ad")

set_target_properties(
        ${TARGET} PROPERTIES
        OUTPUT_NAME "fw"
)

target_link_libraries(
        ${TARGET}
        PUBLIC ${FW_PROJECT_TARGET}
)

add_custom_target(
        fw_hex
        COMMAND ${CMAKE_OBJCOPY} -O ihex -S "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.hex"
        COMMAND size -A -d "${PROJECT_NAME}.hex"
        DEPENDS ${TARGET}
        BYPRODUCTS "${PROJECT_NAME}.hex"
        DEPENDS ${TARGET}
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

add_custom_target(
        fw_bin
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .interrupts -j .flash_config -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin.header"
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .user_rom -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin.user"
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .fast_code -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin.code"
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .text -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin.text"
        COMMAND ${CMAKE_OBJCOPY} -O binary -j .data -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.bin.data"
        BYPRODUCTS "${PROJECT_NAME}.header.bin" "${PROJECT_NAME}.bin.user" "${PROJECT_NAME}.bin.code" "${PROJECT_NAME}.bin.text" "${PROJECT_NAME}.bin.data"
        DEPENDS ${TARGET}
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

set(GZ "gzip")

add_custom_target(
        fw_pkg
        #COMMAND ${GZ} "${PROJECT_NAME}.bin.header"
        #COMMAND ${GZ} "${PROJECT_NAME}.bin.user"
        #COMMAND ${GZ} "${PROJECT_NAME}.bin.code"
        #COMMAND ${GZ} "${PROJECT_NAME}.bin.text"
        #COMMAND ${GZ} "${PROJECT_NAME}.bin.data"
        #COMMAND cat "${PROJECT_NAME}.bin.header.gz" "${PROJECT_NAME}.bin.user.gz" "${PROJECT_NAME}.bin.code.gz" "${PROJECT_NAME}.bin.text.gz" "${PROJECT_NAME}.bin.data.gz" > "${PROJECT_NAME}.bin"
        
        COMMAND cat "${PROJECT_NAME}.bin.header" "${PROJECT_NAME}.bin.user" "${PROJECT_NAME}.bin.code" "${PROJECT_NAME}.bin.text" "${PROJECT_NAME}.bin.data" > "${PROJECT_NAME}.bin"

        BYPRODUCTS "${PROJECT_NAME}.bin"
        DEPENDS fw_bin
        WORKING_DIRECTORY "${BUILD_ROOT}"
)


add_custom_target(
        fw_srec
        COMMAND ${CMAKE_OBJCOPY} -O srec -S  "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.srec"
        COMMAND size -A -d "${PROJECT_NAME}.srec"
        BYPRODUCTS "${PROJECT_NAME}.srec"
        DEPENDS ${TARGET}
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

# https://github.com/energia/Energia/wiki/FLASH-and-estimated-RAM-Usage
# text + data = flash
# bss  + data = ram
add_custom_target(
        fw_siz
        COMMAND size -A -d "$<TARGET_FILE:${TARGET}>" | tee fw.siz
        DEPENDS ${TARGET}
        WORKING_DIRECTORY "${BUILD_ROOT}"
)

add_dependencies(fw_bin fw_siz)
