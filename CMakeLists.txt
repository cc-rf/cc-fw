cmake_minimum_required(VERSION 3.4)
project(kx)
set(ROOT "${CMAKE_CURRENT_LIST_DIR}")

list(APPEND CMAKE_MODULE_PATH "${ROOT}/etc/cmake/")

if(CMAKE_CURRENT_BINARY_DIR STREQUAL "${CMAKE_CURRENT_LIST_DIR}")
    message(FATAL_ERROR "Only out-of-source builds are supported!")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
    message(STATUS "CMAKE_BUILD_TYPE defaulting to Debug")
endif()

#string(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_LOWER)
#set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/out/${CMAKE_BUILD_TYPE_LOWER}")
#set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/out/${CMAKE_BUILD_TYPE_LOWER}")

include(util)

include(fw/config.cmake)
add_subdirectory("fw")

#add_custom_target(kx DEPENDS fw)
