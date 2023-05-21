#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/watchdog.h"

#include "Watchdog.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Watchdog;

Watchdog::Watchdog(MemoryBus &mem) : mem(mem)
{
}

void Watchdog::reset()
{
    ctrl = WATCHDOG_CTRL_RESET;
    tick = WATCHDOG_TICK_RESET;

    clock.reset();

    timer = 0;
    tickCounter = 0;
    ticks = 0;
}

void Watchdog::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);

    if(!passed)
        return;

    if(!(tick & WATCHDOG_TICK_ENABLE_BITS)) // enable tick
    {
        clock.addCycles(passed);
        return;
    }

    // tick counter
    unsigned int tickCycles = tick & WATCHDOG_TICK_CYCLES_BITS;

    int ticksToAdd = 0;

    if(!tickCycles)
        ticksToAdd = passed;
    else
    {
        tickCounter += passed;
        while(tickCounter >= tickCycles)
        {
            tickCounter -= tickCycles;
            ticksToAdd++;
        }
    }

    ticks += ticksToAdd;

    // watchdog timer
    if(ticksToAdd && (ctrl & WATCHDOG_CTRL_ENABLE_BITS)) // timer enable
    {
        ticksToAdd *= 2; // RP2040-E1

        if(ticksToAdd > timer) // oops, missed it
            ticksToAdd = timer;

        timer -= ticksToAdd;

        if(timer == 0)
        {
            //reset
            // TODO: WDSEL
            logf(LogLevel::Info, logComponent, "Watchdog reset!");
            mem.reset();
        }
    }

    clock.addCycles(passed);
}

uint32_t Watchdog::getTicks()
{
    return ticks;
}

uint64_t Watchdog::getTickTarget(uint32_t numTicks)
{
    int tickCycles = tick & WATCHDOG_TICK_CYCLES_BITS;
    uint32_t cycles = numTicks * tickCycles - tickCounter;
    return clock.getTimeToCycles(cycles);
}

uint32_t Watchdog::regRead(uint32_t addr)
{
    switch(addr)
    {
        case WATCHDOG_CTRL_OFFSET:
            return ctrl | timer;
        case WATCHDOG_LOAD_OFFSET:
            return 0;
        case WATCHDOG_REASON_OFFSET:
            return 0;
        case WATCHDOG_SCRATCH0_OFFSET:
        case WATCHDOG_SCRATCH1_OFFSET:
        case WATCHDOG_SCRATCH2_OFFSET:
        case WATCHDOG_SCRATCH3_OFFSET:
        case WATCHDOG_SCRATCH4_OFFSET:
        case WATCHDOG_SCRATCH5_OFFSET:
        case WATCHDOG_SCRATCH6_OFFSET:
        case WATCHDOG_SCRATCH7_OFFSET:
            return scratch[addr / 4 - 3];
        case WATCHDOG_TICK_OFFSET:
            return tick | WATCHDOG_TICK_RUNNING_BITS | tickCounter << 11;
    }
    return 0;
}

void Watchdog::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case WATCHDOG_CTRL_OFFSET:
            updateReg(ctrl, data & 0xC7000000, atomic);
            return;
        case WATCHDOG_LOAD_OFFSET:
            timer = data & 0xFFFFFF;
            return;
        case WATCHDOG_REASON_OFFSET:
            return;
        case WATCHDOG_SCRATCH0_OFFSET:
        case WATCHDOG_SCRATCH1_OFFSET:
        case WATCHDOG_SCRATCH2_OFFSET:
        case WATCHDOG_SCRATCH3_OFFSET:
        case WATCHDOG_SCRATCH4_OFFSET:
        case WATCHDOG_SCRATCH5_OFFSET:
        case WATCHDOG_SCRATCH6_OFFSET:
        case WATCHDOG_SCRATCH7_OFFSET:
            scratch[addr / 4 - 3] = data;
            return;
        case WATCHDOG_TICK_OFFSET:
            updateReg(tick, data & (WATCHDOG_TICK_ENABLE_BITS | WATCHDOG_TICK_CYCLES_BITS), atomic);
            return;
    }
}