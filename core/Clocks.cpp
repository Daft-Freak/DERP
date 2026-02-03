#include <cstdio>

#include "hardware/regs/clocks.h"
#include "hardware/regs/pll.h"

#include "Clocks.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Clocks;

static const char *clockNames[]{"GPOUT0", "GPOUT1", "GPOUT2", "GPOUT3", "REF", "SYS", "PERI", "USB", "ADC", "RTC"};

void Clocks::reset()
{
    for(auto &clk : hw.clk)
    {
        clk.ctrl = CLOCKS_CLK_GPOUT0_CTRL_RESET;
        clk.div = CLOCKS_CLK_GPOUT0_DIV_RESET;
    }

    for(auto &clock : clockFreq)
        clock = 0;

    pllSys.cs = pllUSB.cs = PLL_CS_RESET;
    pllSys.pwr = pllUSB.pwr = PLL_PWR_RESET;
    pllSys.fbdiv_int = pllUSB.fbdiv_int = PLL_FBDIV_INT_RESET;
    pllSys.prim = pllUSB.prim = PLL_PRIM_RESET;

    // calcutate always enabled clocks
    calcFreq(clk_ref);
    calcFreq(clk_sys);
}

void Clocks::adjustClocks()
{
    uint64_t minTime = ~0ull;
    uint64_t maxTime = 0;

    for(auto &clock : targets)
    {
        auto time = clock.second.getTime();

        if(time < minTime)
            minTime = time;
        
        if(time > maxTime)
            maxTime = time;
    }

    // don't do anything until the highest bit is set somewhere  
    if(!(maxTime & (1ull << 63)))
        return;

    for(auto &clock : targets)
        clock.second.adjustTime(minTime);
}

uint32_t Clocks::getClockFrequency(int clock) const
{
    return clockFreq[clock];
}

void Clocks::addClockTarget(int clock, ClockTarget &target)
{
    targets.emplace(clock, target);
}

uint32_t Clocks::regRead(uint32_t addr)
{
    if(addr < CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET)
    {
        // ctrl, div, selected x10
        int clock = addr / 12;
        int reg = addr % 12;

        if(reg == CLOCKS_CLK_GPOUT0_CTRL_OFFSET)
            return hw.clk[clock].ctrl;
        else if(reg == CLOCKS_CLK_GPOUT0_DIV_OFFSET)
            return hw.clk[clock].div;
        else if(reg == CLOCKS_CLK_GPOUT0_SELECTED_OFFSET)
        {
            if(clock == clk_ref)
                return 1 << (hw.clk[clock].ctrl & CLOCKS_CLK_REF_CTRL_SRC_BITS); // TODO: doesn't change instantly
            else if(clock == clk_sys)
                return 1 << (hw.clk[clock].ctrl & CLOCKS_CLK_SYS_CTRL_SRC_BITS);

            return 1;
        }
    }
    else
        logf(LogLevel::NotImplemented, logComponent, "R %04X", addr);

    return 0xBADADD55;
}

void Clocks::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < CLOCKS_CLK_SYS_RESUS_CTRL_OFFSET)
    {
        //ctrl,div,selected x10
        int clock = addr / 12;
        int reg = addr % 12;

        bool changed = false;

        if(reg == CLOCKS_CLK_GPOUT0_CTRL_OFFSET)
            changed = updateReg(hw.clk[clock].ctrl, data, atomic);
        else if(reg == CLOCKS_CLK_GPOUT0_DIV_OFFSET)
            changed = updateReg(hw.clk[clock].div, data, atomic);

        // update clock freq
        if(changed)
        {
            bool enabled = clock == clk_ref || clock == clk_sys || (hw.clk[clock].ctrl & CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
            if(enabled)
                calcFreq(clock);
            else
                clockFreq[clock] = 0;
        }
    }
    else
        logf(LogLevel::NotImplemented, logComponent, "W %04X%s%08X", addr, op[atomic], data);
}

uint32_t Clocks::pllSysRegRead(uint32_t addr)
{
    switch(addr)
    {
        case PLL_CS_OFFSET:
            return pllSys.cs | PLL_CS_LOCK_BITS;
        case PLL_PWR_OFFSET:
            return pllSys.pwr;
        case PLL_FBDIV_INT_OFFSET:
            return pllSys.fbdiv_int;
        case PLL_PRIM_OFFSET:
            return pllSys.prim;
    }

    return 0xBADADD55;
}

void Clocks::pllSysRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case PLL_CS_OFFSET:
            updateReg(pllSys.cs, data & ~PLL_CS_LOCK_BITS, atomic);
            break;
        case PLL_PWR_OFFSET:
            updateReg(pllSys.pwr, data, atomic);
            break;
        case PLL_FBDIV_INT_OFFSET:
            updateReg(pllSys.fbdiv_int, data, atomic);
            break;
        case PLL_PRIM_OFFSET:
            updateReg(pllSys.prim, data, atomic);
            break;
    }

    // (REF / REFDIV) * FBDIV / (POSTDIV1 * POSTDIV2)
    int ref = 12 * 1000 * 1000; // assume 12MHz
    int freq = (ref / (pllSys.cs & 0x3F)) * pllSys.fbdiv_int / (((pllSys.prim >> 16) & 7) * ((pllSys.prim >> 12) & 7));
    logf(LogLevel::Debug, logComponent, "PLL_SYS = (%i / %i) * %i / (%i * %i) = %i", ref, pllSys.cs & 0x3F, pllSys.fbdiv_int, (pllSys.prim >> 16) & 7, (pllSys.prim >> 12) & 7, freq);
}

uint32_t Clocks::pllUSBRegRead(uint32_t addr)
{
    switch(addr)
    {
        case PLL_CS_OFFSET:
            return pllUSB.cs | PLL_CS_LOCK_BITS;
        case PLL_PWR_OFFSET:
            return pllUSB.pwr;
        case PLL_FBDIV_INT_OFFSET:
            return pllUSB.fbdiv_int;
        case  PLL_PRIM_OFFSET:
            return pllUSB.prim;
    }

    return 0xBADADD55;
}

void Clocks::pllUSBRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case PLL_CS_OFFSET:
            updateReg(pllUSB.cs, data & ~PLL_CS_LOCK_BITS, atomic);
            break;
        case PLL_PWR_OFFSET:
            updateReg(pllUSB.pwr, data, atomic);
            break;
        case PLL_FBDIV_INT_OFFSET:
            updateReg(pllUSB.fbdiv_int, data, atomic);
            break;
        case PLL_PRIM_OFFSET:
            updateReg(pllUSB.prim, data, atomic);
            break;
    }

    // (REF / REFDIV) * FBDIV / (POSTDIV1 * POSTDIV2)
    int ref = 12 * 1000 * 1000; // assume 12MHz
    int freq = (ref / (pllSys.cs & 0x3F)) * pllUSB.fbdiv_int / (((pllUSB.prim >> 16) & 7) * ((pllUSB.prim >> 12) & 7));
    logf(LogLevel::Debug, logComponent, "PLL_USB = (%i / %i) * %i / (%i * %i) = %i", ref, pllUSB.cs & 0x3F, pllUSB.fbdiv_int, (pllUSB.prim >> 16) & 7, (pllUSB.prim >> 12) & 7, freq);
}

void Clocks::calcFreq(int clock)
{
    static const int xoscFreq = 12 * 1000 * 1000;
    static const int roscFreq = 6 * 1000 * 1000; // ish

    auto getPLLSysFreq = [this]()
    {
        // TODO: bypass
        // TODO: cache?
        if(!(pllSys.pwr & PLL_PWR_VCOPD_RESET))
        {
            auto refDiv = pllSys.cs & PLL_CS_REFDIV_BITS;
            auto postdiv1 = (pllSys.prim & PLL_PRIM_POSTDIV1_BITS) >> PLL_PRIM_POSTDIV1_LSB;
            auto postdiv2 = (pllSys.prim & PLL_PRIM_POSTDIV2_BITS) >> PLL_PRIM_POSTDIV2_LSB;
            return (xoscFreq / refDiv) * pllSys.fbdiv_int / (postdiv1 * postdiv2);
        }

        return 0u;
    };

    auto getPLLUSBFreq = [this]()
    {
        // TODO: bypass
        // TODO: cache?
        if(!(pllUSB.pwr & PLL_PWR_VCOPD_RESET))
        {
            auto refDiv = pllUSB.cs & PLL_CS_REFDIV_BITS;
            auto postdiv1 = (pllUSB.prim & PLL_PRIM_POSTDIV1_BITS) >> PLL_PRIM_POSTDIV1_LSB;
            auto postdiv2 = (pllUSB.prim & PLL_PRIM_POSTDIV2_BITS) >> PLL_PRIM_POSTDIV2_LSB;
            return (xoscFreq / refDiv) * pllUSB.fbdiv_int / (postdiv1 * postdiv2);
        }

        return 0u;
    };

    auto divClock = [](uint32_t freq, uint32_t div) -> uint32_t
    {
        return (static_cast<uint64_t>(freq) << 8) / div;
    };

    auto oldFreq = clockFreq[clock];

    switch(clock)
    {
        // TODO: GPOUT0-3

        case clk_ref:
        {
            int src = hw.clk[clock].ctrl & CLOCKS_CLK_REF_CTRL_SRC_BITS;
            int clkDiv = (hw.clk[clock].div & CLOCKS_CLK_REF_DIV_INT_BITS) >> CLOCKS_CLK_REF_DIV_INT_LSB; // no frac
            if(!clkDiv)
                clkDiv = 1 << 16;

            if(src == CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH)
                clockFreq[clock] = roscFreq / clkDiv;
            else if(src == CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX)
            {
                int aux = (hw.clk[clock].ctrl & CLOCKS_CLK_REF_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_REF_CTRL_AUXSRC_LSB;
                if(aux == CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB)
                    clockFreq[clock] = getPLLUSBFreq() / clkDiv;
                // else gpin0/1
            }
            else if(src == CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC)
                clockFreq[clock] = xoscFreq / clkDiv;

            break;
        }

        case clk_sys:
        {
            int src = hw.clk[clock].ctrl & CLOCKS_CLK_SYS_CTRL_SRC_BITS;
            int clkDiv = hw.clk[clock].div;

            if(!(clkDiv >> CLOCKS_CLK_SYS_DIV_INT_LSB))
                clkDiv |= 1 << 24;

            if(src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF)
                clockFreq[clock] = divClock(clockFreq[4], clkDiv); // REF
            else // aux
            {
                int aux = (hw.clk[clock].ctrl & CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB;

                if(aux == CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS)
                    clockFreq[clock] = divClock(getPLLSysFreq(), clkDiv);
                else if(aux == CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB)
                    clockFreq[clock] = divClock(getPLLUSBFreq(), clkDiv);
                else if(aux == CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC)
                    clockFreq[clock] = divClock(roscFreq, clkDiv);
                else if(aux == CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC)
                    clockFreq[clock] = divClock(xoscFreq, clkDiv);
                // else gpin0/1
            }

            break;
        }

        case clk_peri:
        {
            int aux = (hw.clk[clock].ctrl & CLOCKS_CLK_PERI_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB;
            // no div

            if(aux == CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS)
                clockFreq[clock] = clockFreq[5]; // SYS
            else if(aux == CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS)
                clockFreq[clock] = getPLLSysFreq();
            else if(aux == CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB)
                clockFreq[clock] = getPLLUSBFreq();
            else if(aux == CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH)
                clockFreq[clock] = roscFreq;
            else if(aux == CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_XOSC_CLKSRC)
                clockFreq[clock] = xoscFreq;
            // else gpin0/1

            break;
        }

        case clk_usb:
        case clk_adc:
#ifndef RP2350
        case clk_rtc:
#endif
        {
            int aux = (hw.clk[clock].ctrl & CLOCKS_CLK_USB_CTRL_AUXSRC_BITS) >> CLOCKS_CLK_USB_CTRL_AUXSRC_LSB;

            // TODO: RTC has fractional divider
            int clkDiv = hw.clk[clock].div >> CLOCKS_CLK_USB_DIV_INT_LSB;

            if(!clkDiv)
                clkDiv = 1 << 16;

            if(aux == CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB)
                clockFreq[clock] = getPLLUSBFreq() / clkDiv;
            else if(aux == CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS)
                clockFreq[clock] = getPLLSysFreq() / clkDiv;
            else if(aux == CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH)
                clockFreq[clock] = roscFreq / clkDiv;
            else if(aux == CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_XOSC_CLKSRC)
                clockFreq[clock] = xoscFreq / clkDiv;
            // else gpin0/1
            break;
        }
    }

    if(clockFreq[clock] != oldFreq)
    {
        // update users of this clock
        auto toUpdate = targets.equal_range(clock);

        for(auto it = toUpdate.first; it != toUpdate.second; ++it)
            it->second.setFrequency(getClockFrequency(clock));

        // debug
        logf(LogLevel::Debug, logComponent, "CLK_%s: %i -> %iHz", clockNames[clock], oldFreq, clockFreq[clock]);
    }
}