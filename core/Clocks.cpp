#include <cstdio>

#include "Clocks.h"

static const char *clockNames[]{"GPOUT0", "GPOUT1", "GPOUT2", "GPOUT3", "REF", "SYS", "PERI", "USB", "ADC", "RTC"};

static void updateReg(uint32_t &oldVal, uint32_t newVal, int atomic)
{
    if(atomic == 0)
        oldVal = newVal;
    else if(atomic == 1)
        oldVal ^= newVal;
    else if(atomic == 2)
        oldVal |= newVal;
    else
        oldVal &= ~newVal;
}

void Clocks::reset()
{
    for(auto &reg : ctrl)
        reg = 0;

    for(auto &reg : div)
        reg = 1 << 8;

    pllSysCS = pllUSBCS = 1;
    pllSysPWR = pllUSBPWR = 0b101101;
    pllSysFBDIV = pllUSBFBDIV = 0;

    pllSysPRIM = pllUSBPRIM = 7 << 16 | 7 << 12;
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