# stub pico-sdk functions
function(pico_add_subdirectory subdir)
    add_subdirectory(${subdir})
endfunction()

function(pico_add_doxygen)
endfunction()

function(pico_add_doxygen_exclude)
endfunction()

function(pico_mirrored_target_link_libraries TARGET SCOPE)
    target_link_libraries(${TARGET} ${SCOPE} ${TARGET}_headers)
    foreach(DEPENDENCY IN LISTS ARGN)
        target_link_libraries(${TARGET}_headers ${SCOPE} ${DEPENDENCY}_headers)
        target_link_libraries(${TARGET} ${SCOPE} ${DEPENDENCY})
    endforeach()
endfunction()

macro(pico_promote_common_scope_vars)
endmacro()

# stub hardware_base
add_library(hardware_base INTERFACE)
add_library(hardware_base_headers INTERFACE)

# include enough to get hardware_regs
add_subdirectory(../pico-sdk/src/rp2040 pico-sdk-rp2040)