#pragma once

#include <cstdint>

class MemoryBus;

class Timer final
{
public:
    Timer(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    bool needUpdateForInterrupts()
    {
        return armed & interruptEnables;
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint64_t time, uint32_t addr);
    void regWrite(uint64_t time, uint32_t addr, uint32_t data);

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
