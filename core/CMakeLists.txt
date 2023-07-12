include(pico-sdk-stub.cmake)

add_library(DERPCore INTERFACE)

target_sources(DERPCore INTERFACE
    ARMv6MCore.cpp
    Clocks.cpp
    DMA.cpp
    GDBServer.cpp
    GPIO.cpp
    I2C.cpp
    Logging.cpp
    MemoryBus.cpp
    PWM.cpp
    Timer.cpp
    UART.cpp
    USB.cpp
    Watchdog.cpp
)

target_include_directories(DERPCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(DERPCore INTERFACE hardware_regs hardware_structs)

# recompiler
add_library(DERPRecompiler INTERFACE)

# common sources
target_sources(DERPRecompiler INTERFACE
    compiler/RecompilerGeneric.cpp
    # builders might be here if they didn't define enums with the same name
)

# detect target arch

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64") # OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64"
    target_sources(DERPRecompiler INTERFACE compiler/X86Builder.cpp compiler/X86Target.cpp)
    target_compile_definitions(DERPRecompiler INTERFACE RECOMPILER_X86)
    set(HAVE_RECOMPILER TRUE)
endif()

if(HAVE_RECOMPILER)
    # enable DMG recompiler
    target_sources(DERPCore INTERFACE
        compiler/ARMv6MRecompiler.cpp
    )
    target_link_libraries(DERPCore INTERFACE DERPRecompiler)
endif()