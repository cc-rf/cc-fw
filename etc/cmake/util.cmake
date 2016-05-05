#add_compile_options(-fastcc -ffreestanding)

macro(subdir_list result curdir)
    file(GLOB children RELATIVE ${curdir} ${curdir}/*)
    set(dirlist "")
    foreach(child ${children})
        if(IS_DIRECTORY ${curdir}/${child})
            list(APPEND dirlist ${child})
        endif()
    endforeach()
    set(${result} ${dirlist})
endmacro()

macro(add_all_subdirs)
    subdir_list(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})
    foreach(subdir ${SUBDIRS})
        add_subdirectory(${subdir})
    endforeach()
endmacro()

macro(tgt_sources)
    file(GLOB_RECURSE SOURCE ${ARGN})
    target_sources(${TARGET} PUBLIC ${SOURCE})
endmacro()

macro(tgt_include_directories)
    target_include_directories(${TARGET} PUBLIC ${ARGN})
endmacro()

macro(tgt_compile_options)
    target_compile_options(${TARGET} PUBLIC ${ARGN})
endmacro()

macro(tgt_compile_definitions)
    target_compile_definitions(${TARGET} PUBLIC ${ARGN})
endmacro()

macro(tgt_link_flags FLAGS)
    set_target_properties(${TARGET} PROPERTIES
            LINK_FLAGS "${LINK_FLAGS} ${FLAGS}")
endmacro()

macro(tgt_link_def LDFILE)
    tgt_link_flags("-T${LDFILE}")
endmacro()

function (module_init MODULE)
    # Assumption: when called from a module top level, CMAKE_CURRENT_LIST_DIR is its own dir.
    #   This sets CURRENT_MODULE_DIR in that scope to the proper value, because CMAKE_CURRENT_LIST_DIR
    #   will correspond to the caller of any functions within that module.
    #   See: http://stackoverflow.com/questions/12802377/in-cmake-how-can-i-find-the-directory-of-an-included-file
    set(CURRENT_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}" PARENT_SCOPE)
endfunction()

#[[https://cmake.org/cmake/help/v3.0/command/if.html:
function(component_define)
    cmake_parse_arguments(
            PARSED_ARGS # prefix of output variables
            "" # list of names of the boolean arguments (only defined ones will be true)
            "NAME" # list of names of mono-valued arguments
            "SRCS;DEPS" # list of names of multi-valued arguments (output variables are lists)
            ${ARGN} # arguments of the function to parse, here we take the all original ones
    )
    # note: if it remains unparsed arguments, here, they can be found in variable PARSED_ARGS_UNPARSED_ARGUMENTS
    if(NOT PARSED_ARGS_NAME)
        message(FATAL_ERROR "You must provide a name")
    endif(NOT PARSED_ARGS_NAME)
    message("Provided sources are:")
    foreach(src ${PARSED_ARGS_SRCS})
        message("- ${src}")
    endforeach(src)
endfunction(my_add_lib)
]]
