#pragma once

#include <cstdint>

class ClockTarget final
{
public:
    void reset()
    {
        emuTime = 0;
    }

    uint64_t getTime() const
    {
        return emuTime;
    }

    void addCycles(uint32_t cycles)
    {
        emuTime += cycles * clockScale;
    }

    void adjustTime(uint64_t base)
    {
        emuTime -= base;
    }

    uint64_t getTargetTime(int ms) const
    {
        return emuTime + (1ull << 63) / 1000 * ms;
    }

    uint32_t getCyclesToTime(uint64_t targetTime, bool round = false) const
    {
        return (targetTime - emuTime + (round ? clockScale - 1 : 0)) / clockScale;
    }

    uint64_t getTimeToCycles(uint32_t cycles) const
    {
        return emuTime + cycles * clockScale;
    }

    void setFrequency(uint32_t freq)
    {
        const uint64_t base = (1ull << 63) - 1;

        clockScale = base / freq;
    }
    
private:

    uint64_t emuTime = 0;
    uint64_t clockScale = 1;
};