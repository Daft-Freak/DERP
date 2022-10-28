#include <cstdio>

#include "DMA.h"

#include "MemoryBus.h"

DMA::DMA(MemoryBus &mem) : mem(mem)
{
}

void DMA::reset()
{
    clock.reset();
}

void DMA::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);

    clock.addCycles(passed);
}

uint32_t DMA::regRead(uint32_t addr)
{
    if(addr < 0x400)
    {
        int ch = addr / 0x40;
        printf("DMA ch %i R %03X\n", ch, addr);
    }
    else
        printf("DMA R %08X\n", addr);
    return 0xBADADD55;
}

void DMA::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < 0x400)
    {
        int ch = addr/ 0x40;
        printf("DMA ch %i W %03X%s%08X\n", ch, addr, op[atomic], data);
    }
    else
        printf("DMA W %03X%s%08X\n", addr, op[atomic], data);
}