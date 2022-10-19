#include <cstdio>

#include "Clocks.h"

uint32_t Clocks::regRead(uint32_t addr)
{
    // boot hack
    if(addr == 0x38 || addr == 0x44)
    {
        printf("R CLK_%s_SELECTED\n", addr == 0x38 ? "REF" : "SYS");

        // will eventually return the right one
        static int i = 0;
        int ret = 1 << i;
        i = (i + 1) % 3;
        return ret;
    }

    printf("CLOCKS R %04X\n", addr);
    return 0xBADADD55;
}

void Clocks::regWrite(uint32_t addr, uint32_t data)
{
    printf("CLOCKS W %04X = %08X\n", addr, data);
}