cmake_minimum_required(VERSION 3.4)
project(cloudchaser)
set(ROOT "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${ROOT}/etc/cmake/")

if(CMAKE_CURRENT_BINARY_DIR STREQUAL "${CMAKE_CURRENT_LIST_DIR}")
    message(FATAL_ERROR "Only out-of-source builds are supported!")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE INTERNAL "Build Type")
    message(STATUS "CMAKE_BUILD_TYPE defaulting to Debug")
endif()

include(util)

include(fw/config.cmake)
add_subdirectory("fw")
