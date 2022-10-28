#pragma once

#include <cstdint>

#include "ClockTarget.h"

class MemoryBus;

class DMA final
{
public:
    DMA(MemoryBus &mem);

    void reset();

    void update(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    MemoryBus &mem;

    ClockTarget clock;
};