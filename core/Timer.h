#pragma once

#include <cstdint>

class MemoryBus;

class Timer final
{
public:
    Timer(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(armed & interruptEnables)
            update(target);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr, uint64_t time);
    void regWrite(uint32_t addr, uint32_t data, uint64_t time);

private:
    MemoryBus &mem;

    uint64_t time;
    uint32_t latchedHighTime, writeLowTime;

    uint32_t alarms[4];
    uint32_t armed;

    uint32_t interrupts;
    uint32_t interruptEnables;
    uint32_t interruptForce;

    uint32_t lastTicks;
};
