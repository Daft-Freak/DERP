#include <cstdio>

#include "GPIO.h"

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

void GPIO::reset()
{
    for(auto &reg : ctrl)
        reg = 0;

    for(auto &reg : interrupts)
        reg = 0;

    for(auto &reg : proc0InterruptEnables)
        reg = 0;
}

uint32_t GPIO::regRead(uint32_t addr)
{
    if(addr < 0xF0)
    {
        if(addr & 4) // GPIOx_STATUS
            return ctrl[addr / 8];
        // else status
    }
    else if(addr < 0x100) // INTR0-3
        return interrupts[(addr - 0xF0) / 4];
    else if(addr < 0x110) // PROC0_INTE0-3
        return proc0InterruptEnables[(addr - 0x100) / 4];

    printf("IO_BANK0 R %04X\n", addr);

    return 0xBADADD55;
}

void GPIO::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < 0xF0)
    {
        if(addr & 4) // GPIOx_STATUS
        {
            updateReg(ctrl[addr / 8], data, atomic);
            return;
        }
        // else status
    }
    else if(addr < 0x100) // INTR0-3
    {
        if(atomic == 0)
        {
            interrupts[(addr - 0xF0) / 4] &= ~(data & 0xCCCCCCCC);
            return;
        }
    }
    else if(addr < 0x110) // PROC0_INTE0-3
    {
        updateReg(proc0InterruptEnables[(addr - 0x100) / 4], data, atomic);
        return;
    }

    printf("IO_BANK0 W %04X%s%08X\n", addr, op[atomic], data);
}

uint32_t GPIO::padsRegRead(uint32_t addr)
{
    printf("PADS_BANK0 R %04X\n", addr);

    return 0xBADADD55;
}

void GPIO::padsRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    printf("PADS_BANK0 W %04X%s%08X\n", addr, op[atomic], data);
}