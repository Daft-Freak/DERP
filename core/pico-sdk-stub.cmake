# stub pico-sdk functions
function(pico_add_library target)
    add_library(${target}_headers INTERFACE)
    add_library(${target} INTERFACE)
    target_link_libraries(${target} INTERFACE ${target}_headers)
endfunction()

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

# include enough to get hardware_regs
add_subdirectory(../pico-sdk/src/host/pico_platform pico-sdk-pico-platform)
add_subdirectory(../pico-sdk/src/common/pico_base pico-sdk-pico-base)
add_subdirectory(../pico-sdk/src/rp2_common/hardware_base pico-sdk-hw-base)
add_subdirectory(../pico-sdk/src/rp2040 pico-sdk-rp2040)