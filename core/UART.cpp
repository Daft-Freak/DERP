#include <cstdio>

#include "UART.h"

#include "MemoryBus.h"

UART::UART(MemoryBus &mem, int index) : mem(mem), index(index)
{
}

void UART::reset()
{
}

uint32_t UART::regRead(uint32_t addr)
{
    printf("UART%i R %04X\n", index, addr);

    return 0xBADADD55;
}

void UART::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    printf("UART%i W %04X%s%08X\n", index, addr, op[atomic], data);
}
