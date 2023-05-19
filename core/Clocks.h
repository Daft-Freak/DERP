#pragma once

#include <cstdint>
#include <map>

#include "hardware/structs/clocks.h"
#include "hardware/structs/pll.h"

#include "ClockTarget.h"

class Clocks final
{
public:
    void reset();

    void adjustClocks();

    uint32_t getClockFrequency(int clock) const;

    void addClockTarget(int clock, ClockTarget &target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    // also handle the PLLs here
    uint32_t pllSysRegRead(uint32_t addr);
    void pllSysRegWrite(uint32_t addr, uint32_t data);

    uint32_t pllUSBRegRead(uint32_t addr);
    void pllUSBRegWrite(uint32_t addr, uint32_t data);

private:
    void calcFreq(int clock);

    clocks_hw_t hw;

    uint32_t clockFreq[CLK_COUNT];

    pll_hw_t pllSys, pllUSB;

    std::multimap<int, ClockTarget &> targets;
};