#include <cstdio>

#include "USB.h"

#include "MemoryBus.h"

USB::USB(MemoryBus &mem) : mem(mem)
{
}

void USB::reset()
{

}

uint32_t USB::regRead(uint32_t addr)
{
    switch(addr)
    {
    }

    printf("USB R %04X\n", addr);

    return 0xBADADD55;
}

void USB::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
    }

    printf("USB W %04X%s%08X\n", addr, op[atomic], data);
}
