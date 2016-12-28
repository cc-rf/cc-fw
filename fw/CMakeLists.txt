cmake_minimum_required(VERSION 3.4)
project(fw C ASM)
set(FW_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include(armgcc)

set(CFLAGS_STD "-std=gnu11 -Wall")
set(CFLAGS_STD_DEBUG "-O0 -g3 -D_DEBUG=1 -DDEBUG")
set(CFLAGS_STD_RELEASE "-Os -g0 -D_DEBUG=0")
set(CFLAGS_FW "-flto")

set(CFLAGS_KINETIS "-fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" )
set(CFLAGS_CPU_CM4F "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" )

# rdimon is for SWO/Semihosting, nosys is for without
# NOTE: what about rdpmon? and redboot?
#set(SPECS "-specs=rdimon.specs -specs=nano.specs")
set(SPECS "-specs=nosys.specs -specs=nano.specs")
set(LFLAGS_FW "-Wl,-flto -flto -Wl,--gc-sections -Wl,-Map,\"${PROJECT_NAME}.map\" -Wl,-z,muldefs -Wl,--no-wchar-size-warning ${SPECS}")

set(CFLAGS "${CFLAGS_STD} ${CFLAGS_KINETIS} ${CFLAGS_CPU_CM4F}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CFLAGS}" CACHE INTERNAL "C Flags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS}" CACHE INTERNAL "CXX Flags")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE INTERNAL "C Flags (DEBUG)")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE INTERNAL "CXX Flags (DEBUG)")
set(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}" CACHE INTERNAL "ASM Flags (DEBUG)")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "C Flags (RELEASE)")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "CXX Flags (RELEASE)")
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}" CACHE INTERNAL "ASM Flags (RELEASE)")

if(NOT DEFINED CONFIG_FW_PROJECT)
    message(FATAL_ERROR "CONFIG_FW_PROJECT is required")
endif()

set(FW_PROJECT_TARGET fw_project_${CONFIG_FW_PROJECT})
set(FW_PROJECT_PATH "project/${CONFIG_FW_PROJECT}")

#set(EXECUTABLE_OUTPUT_PATH "${EXECUTABLE_OUTPUT_PATH}/fw/${FW_PROJECT_NAME}")
#set(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_PATH}/fw/${FW_PROJECT_NAME}/lib")

#TODO: Check that TARGET is set? or maybe keep as "firmware" always

include("${FW_PROJECT_PATH}/config.cmake")

if(DEFINED CONFIG_FW_BOARD)
    set(FW_BOARD_PATH "board/${CONFIG_FW_BOARD}")
    include("${FW_BOARD_PATH}/config.cmake")
endif()

add_subdirectory(board)
add_subdirectory(kinetis)
add_subdirectory(project)
add_subdirectory(system)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LFLAGS_FW} ${CPU_LDFILE}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${LFLAGS_FW}")

set(TARGET "${PROJECT_NAME}")
add_executable(${TARGET} fw.c)
target_compile_options(${TARGET} PUBLIC ${CFLAGS_FW})
target_link_libraries(${TARGET} PRIVATE fw_project)

add_custom_target(
        fw_hex
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${TARGET}>" "${PROJECT_NAME}.hex"
        BYPRODUCTS "${PROJECT_NAME}.hex"
)


# https://github.com/energia/Energia/wiki/FLASH-and-estimated-RAM-Usage
# text + data = flash
# bss  + data = ram
add_custom_target(
        fw_siz
        COMMAND ${CMAKE_OBJSIZE} --format=berkeley --totals "$<TARGET_FILE:${TARGET}>"
)

add_dependencies(fw_hex ${TARGET})
