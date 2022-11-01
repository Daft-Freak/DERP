#pragma once

#include <cstdint>

#include "ClockTarget.h"

class Watchdog final
{
public:
    Watchdog();

    void reset();

    void update(uint64_t target);

    uint32_t getTicks();

    uint64_t getTickTarget(uint32_t numTicks);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    uint32_t ctrl;
    uint32_t scratch[8];
    uint32_t tick;

    ClockTarget clock;

    int timer;
    unsigned int tickCounter;
    uint32_t ticks;
};