#pragma once

#include <cstdint>

#include "hardware/structs/pio.h"

#include "ClockTarget.h"

class MemoryBus;

class PIO final
{
public:
    PIO(MemoryBus &mem, int index);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(hw.inte0 || hw.inte1)
            update(target);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    MemoryBus &mem;
    int index;

    ClockTarget clock;

    pio_hw_t hw;
};
