#pragma once

#include <cstdint>

class MemoryBus;

class UART final
{
public:
    UART(MemoryBus &mem, int index);

    void reset();

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

private:
    MemoryBus &mem;
    int index;

    uint32_t baudInt;
    uint32_t baudFrac;
    uint32_t lcr;
    uint32_t cr;

    int txDataOff;
    uint8_t txData[1024];
};