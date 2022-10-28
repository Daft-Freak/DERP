#include <cstdio>

#include "GPIO.h"

#include "MemoryBus.h"

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

GPIO::GPIO(MemoryBus &mem) : mem(mem)
{
}

void GPIO::reset()
{
    for(auto &reg : ctrl)
        reg = 0;

    for(auto &reg : interrupts)
        reg = 0;

    for(auto &reg : proc0InterruptEnables)
        reg = 0;

    inputs = 0;
}

void GPIO::setInputs(uint32_t inputs)
{
    // TODO: handle floating inputs?

    uint32_t changed = inputs ^ this->inputs;

    if(changed)
    {
        for(int i = 0; i < 32; i++)
        {
            if(!(changed & (1 << i)))
                continue;

            // TODO: proc1
            int ioShift = i % 8 * 4;
            auto p0IntEn = (proc0InterruptEnables[i / 8] >> ioShift) & 0xF;

            bool newState = inputs & (1 << i);
            bool oldState = this->inputs & (1 << i);

            // TODO: level should stick
            if(!newState && (p0IntEn & (1 << 0))) // LEVEL_LOW
                interrupts[i / 8] |= 1 << (ioShift + 0);
            else if(newState && (p0IntEn & (1 << 1))) // LEVEL_HIGH
                interrupts[i / 8] |= 1 << (ioShift + 1);
            else if(!newState && oldState && (p0IntEn & (1 << 2))) // EDGE_LOW
                interrupts[i / 8] |= 1 << (ioShift + 2);
            else if(newState && !oldState && (p0IntEn & (1 << 3))) // EDGE_HIGH
                interrupts[i / 8] |= 1 << (ioShift + 3);

            if(interrupts[i / 8] & proc0InterruptEnables[i / 8])
            {
                // TODO: only to one core
                mem.setPendingIRQ(13);// IO_IRQ_BANK0
            }
        }
    }

    this->inputs = inputs;
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
    else if(addr >= 0x120 && addr < 0x130) // PROC0_INTS0-3
    {
        int index = (addr - 0x120) / 4;
        // TODO: force
        return interrupts[index] & proc0InterruptEnables[index];
    }

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