#include "Watchdog.h"

#include "MemoryBus.h"

Watchdog::Watchdog()
{
}

void Watchdog::reset()
{
    ctrl = 7 << 24;
    tick = 1 << 9;

    clock.reset();

    timer = 0;
    tickCounter = 0;
    ticks = 0;
}

void Watchdog::update(uint64_t target)
{
    // TODO: actual watchdog

    auto passed = clock.getCyclesToTime(target);

    if(!passed)
        return;

    if(!(tick & (1 << 9)))
    {
        clock.addCycles(passed);
        return;
    }

    unsigned int tickCycles = tick & 0x1FF;

    if(!tickCycles)
        ticks += passed;
    else
    {
        tickCounter += passed;
        while(tickCounter >= tickCycles)
        {
            tickCounter -= tickCycles;
            ticks++;
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
    int tickCycles = tick & 0x1FF;
    uint32_t cycles = numTicks * tickCycles - tickCounter;
    return clock.getTimeToCycles(cycles);
}

uint32_t Watchdog::regRead(uint32_t addr)
{
    switch(addr)
    {
        case 0: // CTRL
            return ctrl | timer;
        case 4: // LOAD
            return 0;
        case 8: // REASON
            return 0;
        case 0x0C: // SCRATCH0
        case 0x10:
        case 0x14:
        case 0x18:
        case 0x1C:
        case 0x20:
        case 0x24:
        case 0x28: // SCRATCH7
            return scratch[addr / 4 - 3];
        case 0x2C: // TICK
            return tick | (1 << 10) | tickCounter << 11;
    }
    return 0;
}

void Watchdog::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case 0: // CTRL
            updateReg(ctrl, data & 0xC7000000, atomic);
            return;
        case 4: // LOAD
            timer = data & 0xFFFFFF;
            return;
        case 8: // REASON
            return;
        case 0x0C: // SCRATCH0
        case 0x10:
        case 0x14:
        case 0x18:
        case 0x1C:
        case 0x20:
        case 0x24:
        case 0x28: // SCRATCH7
            scratch[addr / 4 - 3] = data;
            return;
        case 0x2C: // TICK
            updateReg(tick, data & 0x3FF, atomic);
            return;
    }
}