#include <cstdio>

#include "Clocks.h"

static const char *clockNames[]{"GPOUT0", "GPOUT1", "GPOUT2", "GPOUT3", "REF", "SYS", "PERI", "USB", "ADC", "RTC"};

static bool updateReg(uint32_t &curVal, uint32_t newVal, int atomic)
{
    auto oldVal = curVal;

    if(atomic == 0)
        curVal = newVal;
    else if(atomic == 1)
        curVal ^= newVal;
    else if(atomic == 2)
        curVal |= newVal;
    else
        curVal &= ~newVal;

    return curVal != oldVal;
}

void Clocks::reset()
{
    for(auto &reg : ctrl)
        reg = 0;

    for(auto &reg : div)
        reg = 1 << 8;

    for(auto &clock : clockVal)
        clock = 0;

    pllSysCS = pllUSBCS = 1;
    pllSysPWR = pllUSBPWR = 0b101101;
    pllSysFBDIV = pllUSBFBDIV = 0;

    pllSysPRIM = pllUSBPRIM = 7 << 16 | 7 << 12;
}

void Clocks::update(int ms)
{
    // this is going to be wrong if the CPU reconfigures a clock

    const int xoscFreq = 12 * 1000 * 1000;
    const int roscFrec = 6 * 1000 * 1000; // ish

    int pllSysFreq = 0, pllUSBFreq = 0;
    
    // TODO: bypass
    if(!(pllSysPWR & 1))
        pllSysFreq = (xoscFreq / (pllSysCS & 0x3F)) * pllSysFBDIV / (((pllSysPRIM >> 16) & 7) * ((pllSysPRIM >> 12) & 7));
    if(!(pllUSBPWR & 1))
        pllUSBFreq = (xoscFreq / (pllSysCS & 0x3F)) * pllUSBFBDIV / (((pllUSBPRIM >> 16) & 7) * ((pllUSBPRIM >> 12) & 7));

    auto oldRef = clockVal[4];
    auto oldSys = clockVal[5];

    // REF
    int src = ctrl[4] & 3;
    int clkDiv = div[4] >> 8;
    int freq = 0;

    if(!clkDiv)
        clkDiv = 1 << 16;

    if(src == 0) // ROSC
        freq = roscFrec;
    else if(src == 1) // aux
    {
        int aux = (ctrl[4] >> 5) & 0x3;
        if(aux == 0)
            freq = pllUSBFreq;
        // else gpin0/1
    }
    else if(src == 2)
        freq = xoscFreq;

    clockVal[4] += (freq / clkDiv) * ms / 1000;

    // USB/ADC/RTC
    for(int i = 7; i < 10; i++)
    {
        if(!(ctrl[i] & (1 << 11)/*ENABLE*/))
            continue;

        freq = 0;
     
        int aux = (ctrl[i] >> 5) & 0x7;

        // TODO: RTC has fractional divider
        clkDiv = div[i] >> 8;

        if(!clkDiv)
            clkDiv = 1 << 16;

        if(aux == 0)
            freq = pllUSBFreq;
        else if(aux == 1)
            freq = pllSysFreq;
        else if(aux == 2)
            freq = roscFrec;
        else if(aux == 3)
            freq = xoscFreq;
        // else gpin0/1

        clockVal[i] += (freq / clkDiv) * ms / 1000;
    }

    // SYS
    src = ctrl[5] & 3;
    clkDiv = div[5];

    if(!(clkDiv >> 8))
        clkDiv |= 1 << 24;

    if(src == 0) // CLK_REF
        clockVal[5] += static_cast<uint64_t>((clockVal[4] - oldRef) << 8) / clkDiv;
    else // aux
    {
        int aux = (ctrl[5] >> 5) & 0x7;
        freq = 0;

        if(aux == 0)
            freq = pllSysFreq;
        else if(aux == 1)
            freq = pllUSBFreq;
        else if(aux == 2)
            freq = roscFrec;
        else if(aux == 3)
            freq = xoscFreq;
        // else gpin0/1

        clockVal[5] += ((static_cast<uint64_t>(freq) << 8) / clkDiv) * ms / 1000;
    }

    // PERI
    if(ctrl[6] & (1 << 11))
    {
        int aux = (ctrl[6] >> 5) & 0x7;
        // no div

        if(aux == 0)
            clockVal[6] += clockVal[5] - oldSys;
        else
        {
            freq = 0;
            if(aux == 1)
                freq = pllSysFreq;
            else if(aux == 2)
                freq = pllUSBFreq;
            else if(aux == 3)
                freq = roscFrec;
            else if(aux == 4)
                freq = xoscFreq;
            // else gpin0/1

            clockVal[6] += freq * ms / 1000;
        }
    }

    // TODO: GPOUTx
}

uint32_t Clocks::getClockVal(int clock)
{
    return clockVal[clock];
}

uint32_t Clocks::regRead(uint32_t addr)
{
    if(addr < 0x78)
    {
        // ctrl, div, selected x10
        int clock = addr / 12;
        int reg = addr / 4 % 3;

        if(reg == 0) // ctrl
            return ctrl[clock];
        else if(reg == 1) // div
            return div[clock];
        else if(reg == 2) // selected
        {
            if(clock == 4) // REF
                return 1 << (ctrl[clock] & 0x3); // TODO: doesn't change instantly
            else if(clock == 5) // SYS
                return 1 << (ctrl[clock] & 1);

            return 1;
        }
    }
    else
        printf("CLOCKS R %04X\n", addr);

    return 0xBADADD55;
}

void Clocks::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < 0x78)
    {
        //ctrl,div,selected x10
        int clock = addr / 12;
        int reg = addr / 4 % 3;

        if(reg == 0) // ctrl
            updateReg(ctrl[clock], data, atomic);
        else if(reg == 1) // div
            updateReg(div[clock], data, atomic);

        int src = ctrl[clock] & 3;
        int frac = (div[clock] & 0xFF) * 1000 / 256;

        if(clock == 4 && src != 1)
            printf("CLK_%s = src %s / %i.%03i\n", clockNames[clock], src == 0 ? "ROSC" : "XOSC", div[clock] >> 8, frac);
        else if(clock == 5 && src == 0)
            printf("CLK_%s = src CLK_REF / %i.%03i\n", clockNames[clock], div[clock] >> 8, frac);
        else
        {
            static const char *auxSrc[][11]
            {
                {"PLL_SYS", "GPIN0", "GPIN1", "PLL_USB", "ROSC_PH", "XOSC", "CLK_SYS", "CLK_USB", "CLK_ADC", "CLK_RTC", "CLK_REF"},
                {"PLL_SYS", "GPIN0", "GPIN1", "PLL_USB", "ROSC_PH", "XOSC", "CLK_SYS", "CLK_USB", "CLK_ADC", "CLK_RTC", "CLK_REF"},
                {"PLL_SYS", "GPIN0", "GPIN1", "PLL_USB", "ROSC_PH", "XOSC", "CLK_SYS", "CLK_USB", "CLK_ADC", "CLK_RTC", "CLK_REF"},
                {"PLL_SYS", "GPIN0", "GPIN1", "PLL_USB", "ROSC_PH", "XOSC", "CLK_SYS", "CLK_USB", "CLK_ADC", "CLK_RTC", "CLK_REF"},
                {"PLL_USB", "GPIN0", "GPIN1"},
                {"PLL_SYS", "PLL_USB", "ROSC", "XOSC", "GPIN0", "GPIN1"},
                {"CLK_SYS", "PLL_SYS", "PLL_USB", "ROSC_PH", "XOSC", "GPIN0", "GPIN1"},
                {"PLL_USB", "PLL_SYS", "ROSC_PH", "XOSC", "GPIN0", "GPIN1"},
                {"PLL_USB", "PLL_SYS", "ROSC_PH", "XOSC", "GPIN0", "GPIN1"},
                {"PLL_USB", "PLL_SYS", "ROSC_PH", "XOSC", "GPIN0", "GPIN1"},
            };
            printf("CLK_%s = aux %s / %i.%03i\n", clockNames[clock], auxSrc[clock][(ctrl[clock] >> 5) & 0xF], div[clock] >> 8, frac);
        }
    }
    else
        printf("CLOCKS W %04X%s%08X\n", addr, op[atomic], data);
}

uint32_t Clocks::pllSysRegRead(uint32_t addr)
{
    switch(addr)
    {
        case 0x0: // CS
            return pllSysCS | (1 << 31)/*LOCK*/;
        case 0x4: // PWR
            return pllSysPWR;
        case 0x8: // FBDIV_INT
            return pllSysFBDIV;
        case 0xC: // PRIM
            return pllSysPRIM;
    }

    return 0xBADADD55;
}

void  Clocks::pllSysRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case 0x0: // CS
            updateReg(pllSysCS, data & ~(1 << 31), atomic);
            break;
        case 0x4: // PWR
            updateReg(pllSysPWR, data, atomic);
            break;
        case 0x8: // FBDIV_INT
            updateReg(pllSysFBDIV, data, atomic);
            break;
        case 0xC: // PRIM
            updateReg(pllSysPRIM, data, atomic);
            break;
    }

    // (REF / REFDIV) * FBDIV / (POSTDIV1 * POSTDIV2)
    int ref = 12 * 1000 * 1000; // assume 12MHz
    int freq = (ref / (pllSysCS & 0x3F)) * pllSysFBDIV / (((pllSysPRIM >> 16) & 7) * ((pllSysPRIM >> 12) & 7));
    printf("PLL_SYS = (%i / %i) * %i / (%i * %i) = %i\n", ref, pllSysCS & 0x3F, pllSysFBDIV, (pllSysPRIM >> 16) & 7, (pllSysPRIM >> 12) & 7, freq);
}

uint32_t Clocks::pllUSBRegRead(uint32_t addr)
{
    switch(addr)
    {
        case 0x0: // CS
            return pllUSBCS | (1 << 31)/*LOCK*/;
        case 0x4: // PWR
            return pllUSBPWR;
        case 0x8: // FBDIV_INT
            return pllUSBFBDIV;
        case 0xC: // PRIM
            return pllUSBPRIM;
    }

    return 0xBADADD55;
}

void  Clocks::pllUSBRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    switch(addr)
    {
        case 0x0: // CS
            updateReg(pllUSBCS, data & ~(1 << 31), atomic);
            break;
        case 0x4: // PWR
            updateReg(pllUSBPWR, data, atomic);
            break;
        case 0x8: // FBDIV_INT
            updateReg(pllUSBFBDIV, data, atomic);
            break;
        case 0xC: // PRIM
            updateReg(pllUSBPRIM, data, atomic);
            break;
    }

    // (REF / REFDIV) * FBDIV / (POSTDIV1 * POSTDIV2)
    int ref = 12 * 1000 * 1000; // assume 12MHz
    int freq = (ref / (pllSysCS & 0x3F)) * pllUSBFBDIV / (((pllUSBPRIM >> 16) & 7) * ((pllUSBPRIM >> 12) & 7));
    printf("PLL_USB = (%i / %i) * %i / (%i * %i) = %i\n", ref, pllUSBCS & 0x3F, pllUSBFBDIV, (pllUSBPRIM >> 16) & 7, (pllUSBPRIM >> 12) & 7, freq);
}