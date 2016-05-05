cmake_minimum_required(VERSION 3.4)
project(fw C ASM)
set(FW_ROOT "${CMAKE_CURRENT_LIST_DIR}")

include(armgcc)

set(CFLAGS_STD "-std=gnu11 -Wall")
set(CFLAGS_STD_DEBUG "-O0 -g3 -D_DEBUG=1 -DDEBUG")
set(CFLAGS_STD_RELEASE "-Os -g0 -D_DEBUG=0")

set(CFLAGS_KINETIS "-fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections" )
set(CFLAGS_CPU_CM4F "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16" )

# rdimon is for SWO/Semihosting, nosys is for without
# NOTE: what about rdpmon? and redboot?
#set(SPECS "-specs=rdimon.specs -specs=nano.specs")
set(SPECS "-specs=nosys.specs -specs=nano.specs")
set(LFLAGS_FW "-Wl,--gc-sections -Wl,-Map,\"${PROJECT_NAME}.map\" -Wl,-z,muldefs -Wl,--no-wchar-size-warning ${SPECS}")

set(CFLAGS "${CFLAGS_STD} ${CFLAGS_KINETIS} ${CFLAGS_CPU_CM4F}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CFLAGS}")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}")
set(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} ${CFLAGS_STD_DEBUG}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}")
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} ${CFLAGS_STD_RELEASE}")

if(NOT DEFINED FW_PROJECT_NAME)
    message(FATAL_ERROR "FW_PROJECT_NAME is required")
endif()

set(FW_PROJECT_TARGET fw_project_${FW_PROJECT_NAME})
set(FW_PROJECT_PATH "project/${FW_PROJECT_NAME}")

#set(EXECUTABLE_OUTPUT_PATH "${EXECUTABLE_OUTPUT_PATH}/fw/${FW_PROJECT_NAME}")
#set(LIBRARY_OUTPUT_PATH "${LIBRARY_OUTPUT_PATH}/fw/${FW_PROJECT_NAME}/lib")

#TODO: Check that TARGET is set? or maybe keep as "firmware" always

include("${FW_PROJECT_PATH}/config.cmake")

if(DEFINED FW_BOARD_NAME)
    set(FW_BOARD_PATH "core/board/${FW_BOARD_NAME}")
    include("${FW_BOARD_PATH}/config.cmake")
endif()

add_subdirectory(core)
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

add_dependencies(fw_hex ${TARGET})