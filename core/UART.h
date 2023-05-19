#pragma once

#include <cstdint>

#include "hardware/structs/uart.h"

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

    uart_hw_t hw;

    int txDataOff;
    uint8_t txData[1024];
};