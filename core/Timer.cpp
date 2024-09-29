#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/timer.h"
#include "hardware/regs/intctrl.h"

#include "Timer.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Timer;

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
    interruptForce = 0;
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
                armed &= ~(1 << i);

                if(interruptEnables & (1 << i))
                    mem.setPendingIRQ(TIMER_IRQ_0 + i); // TODO: should stay pending if not cleared?
            }
        }
    }
}

uint64_t Timer::getNextInterruptTime(uint64_t target)
{
    if(!armed || !interruptEnables)
        return target;

    // assumes watchdog is up to date
    auto &watchdog = mem.getWatchdog();

    for(int i = 0; i < 4; i++)
    {
        if(!(armed & (1 << i)) || !(interruptEnables & (1 << i)))
            continue;

        int ticksToIntr = alarms[i] - (time & 0xFFFFFFFF);

        auto tickTarget = watchdog.getTickTarget(ticksToIntr);

        // if the clock has overflowed, the event if far enough in the future to not worry about yet
        if(tickTarget < watchdog.getClock().getTime())
            continue;

        if(tickTarget < target)
            target = tickTarget;
    }

    return target;
}

uint32_t Timer::regRead(uint32_t addr)
{
    switch(addr)
    {
        case TIMER_TIMEHR_OFFSET:
            return latchedHighTime;
        case TIMER_TIMELR_OFFSET:
            latchedHighTime = time >> 32;
            return time & 0xFFFFFFFF;
        case TIMER_ALARM0_OFFSET:
        case TIMER_ALARM1_OFFSET:
        case TIMER_ALARM2_OFFSET:
        case TIMER_ALARM3_OFFSET:
            return alarms[(addr - TIMER_ALARM0_OFFSET) / 4];
        case TIMER_ARMED_OFFSET:
            return armed;
        case TIMER_TIMERAWH_OFFSET:
            return time >> 32;
        case TIMER_TIMERAWL_OFFSET:
            return time & 0xFFFFFFFF;
        case TIMER_INTE_OFFSET:
            return interruptEnables;
    }

    logf(LogLevel::NotImplemented, logComponent, "R %04X", addr);
    return 0;
}

void Timer::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case TIMER_TIMEHW_OFFSET:
        {
            uint32_t h = time;
            updateReg(h, data, atomic);
            time = static_cast<uint64_t>(h) << 32 | writeLowTime;
            return;
        }
        case TIMER_TIMELW_OFFSET:
            updateReg(writeLowTime, data, atomic);
            return;
        case TIMER_ALARM0_OFFSET:
        case TIMER_ALARM1_OFFSET:
        case TIMER_ALARM2_OFFSET:
        case TIMER_ALARM3_OFFSET:
        {
            int alarm = (addr - 0x10) / 4;
            updateReg(alarms[alarm], data, atomic);
            armed |= 1 << alarm;
            return;
        }
        case TIMER_ARMED_OFFSET:
            // what does atomic + WC do?
            if(!atomic)
            {
                armed &= ~data;
                return;
            }
            break;
        case TIMER_INTR_OFFSET:
            if(!atomic)
            {
                interrupts &= ~data;
                return;
            }
            break;
        case TIMER_INTE_OFFSET:
            updateReg(interruptEnables, data, atomic);
            return;
        case TIMER_INTF_OFFSET:
            updateReg(interruptForce, data, atomic);
            if(interruptForce)
            {
                logf(LogLevel::NotImplemented, logComponent, "Forced timer intr %x", interruptForce); // TODO

                // set any forced irqs pending now (still wrong as it shouldn't clear, but better than nothing)
                auto mask = interruptForce & interruptEnables;

                for(unsigned i = 0; i < NUM_TIMERS; i++)
                {
                    if(mask & (1 << i))
                        mem.setPendingIRQ(TIMER_IRQ_0 + i);
                }
            }
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "W %04X = %08X", addr, data);
}
