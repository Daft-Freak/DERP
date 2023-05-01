#include <cstdio>

#include "UART.h"

#include "MemoryBus.h"

UART::UART(MemoryBus &mem, int index) : mem(mem), index(index)
{
}

void UART::reset()
{
    baudInt = 0;
    baudFrac = 0;
    lcr = 0;
    cr = 0x301; // TXE | RXE

    txDataOff = 0;
}

uint32_t UART::regRead(uint32_t addr)
{
    switch(addr)
    {
        case 0x18: //UARTFR
            return 0b10010000; // FIFOs empty
        case 0x24: // UARTIBRD
            return baudInt;
        case 0x28: // UARTFBRD
            return baudFrac;
        case 0x2C: // UARTLCR_H
            return lcr;
        case 0x30: // UARCR
            return cr;
    }

    printf("UART%i R %04X\n", index, addr);

    return 0xBADADD55;
}

void UART::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case 0x0: // UARTDR
            txData[txDataOff++] = data;
            if(data == '\n')
            {
                printf("UART%i: %.*s", index, txDataOff, txData);
                txDataOff = 0;
            }
            else if(txDataOff == sizeof(txData))
            {
                txDataOff = 0;
                printf("Dropping UART%i data\n", index);
            }
            return;
        case 0x24: // UARTIBRD
            updateReg(baudInt, data, atomic);
            return;
        case 0x28: // UARTFBRD
            updateReg(baudFrac, data, atomic);
            return;
        case 0x2C: // UARTLCR_H
            updateReg(lcr, data, atomic);
            return;
        case 0x30: // UARTCR
            updateReg(cr, data, atomic);
            return;
    }

    printf("UART%i W %04X%s%08X\n", index, addr, op[atomic], data);
}
