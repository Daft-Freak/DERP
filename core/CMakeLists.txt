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
    PIO.cpp
    PWM.cpp
    Timer.cpp
    UART.cpp
    USB.cpp
    Watchdog.cpp
)

target_include_directories(DERPCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(DERPCore INTERFACE hardware_regs hardware_structs)

if(RP2350)
    target_compile_definitions(DERPCore INTERFACE RP2350)
endif()