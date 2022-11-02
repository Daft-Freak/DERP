#include <cstdio>

#include "Timer.h"

#include "MemoryBus.h"

Timer::Timer(MemoryBus &mem) : mem(mem)
{
}

void Timer::reset()
{
    time = 0;
    lastTicks = 0;

    for(auto &al : alarms)
        al = 0;

    armed = 0;

    interrupts = 0;
    interruptEnables = 0;
}

void Timer::update(uint64_t target)
{
    auto &watchdog = mem.getWatchdog();
    watchdog.update(target);

    auto ticks = watchdog.getTicks() - lastTicks;

    if(!ticks)
        return;

    lastTicks = watchdog.getTicks();

    while(ticks)
    {
        auto inc = ticks;
        uint32_t lowTime = time;

        // limit to next alarm
        for(int i = 0; i < 4; i++)
        {
            if(armed & (1 << i))
                inc = std::min(inc, alarms[i] - lowTime);
        }

        ticks -= inc;
        time += inc;

        // check for alarms
        for(int i = 0; i < 4; i++)
        {
            if(!(armed & (1 << i)))
                continue;
            
            if(alarms[i] == (time & 0xFFFFFFFF))
            {
                interrupts |= (1 << i);

                if(interruptEnables & (1 << i))
                    mem.setPendingIRQ(i); // TODO: should stay pending if not cleared?

                armed &= ~(1 << i);
            }
        }
    }
}

void Timer::updateForInterrupts(uint64_t target)
{
    if(armed & interruptEnables)
        update(target);
}

uint64_t Timer::getNextInterruptTime(uint64_t target)
{
    if(!armed || !interruptEnables)
        return target;

    // assumes watchdog is up to date

    for(int i = 0; i < 4; i++)
    {
        if(!(armed & (1 << i)) || !(interruptEnables & (1 << i)))
            continue;

        int ticksToIntr = alarms[i] - (time & 0xFFFFFFFF);

        auto tickTarget = mem.getWatchdog().getTickTarget(ticksToIntr);

        if(tickTarget < target)
            target = tickTarget;
    }

    return target;
}

uint32_t Timer::regRead(uint32_t addr)
{
    switch(addr)
    {
        case 8: // TIMEHR
            return latchedHighTime;
        case 0xC: // TIMELR
            latchedHighTime = time >> 32;
            return time & 0xFFFFFFFF;
        case 0x10: // ALARM0
        case 0x14:
        case 0x18:
        case 0x1C: // ALARM3
            return alarms[(addr - 0x10) / 4];
        case 0x20: // ARMED
            return armed;
        case 0x24: // TIMERAWH
            return time >> 32;
        case 0x28: // TIMERAWL
            return time & 0xFFFFFFFF;
        case 0x38: // INTE
            return interruptEnables;
    }

    printf("TIMER R %04X\n", addr);
    return 0;
}

void Timer::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case 0: // TIMEHW
        {
            uint32_t h = time;
            updateReg(h, data, atomic);
            time = static_cast<uint64_t>(h) << 32 | writeLowTime;
            return;
        }
        case 4: // TIMELW
            updateReg(writeLowTime, data, atomic);
            return;
        case 0x10: // ALARM0
        case 0x14:
        case 0x18:
        case 0x1C: // ALARM3
        {
            int alarm = (addr - 0x10) / 4;
            updateReg(alarms[alarm], data, atomic);
            armed |= 1 << alarm;
            return;
        }
        case 0x20: // ARMED
            // what does atomic + WC do?
            if(!atomic)
            {
                armed &= ~data;
                return;
            }
            break;
        case 0x34: // INTR
            if(!atomic)
            {
                interrupts &= ~data;
                return;
            }
            break;
        case 0x38: // INTE
            updateReg(interruptEnables, data, atomic);
            return;
    }

    printf("TIMER W %04X = %08X\n", addr, data);
}
