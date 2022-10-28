add_library(DERPCore INTERFACE)

target_sources(DERPCore INTERFACE
    ARMv6MCore.cpp
    Clocks.cpp
    DMA.cpp
    GPIO.cpp
    MemoryBus.cpp
)

target_include_directories(DERPCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})
