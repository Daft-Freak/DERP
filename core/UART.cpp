#include <cstdio>

#include "UART.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::UART;

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

    logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);

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
                logf(LogLevel::Info, logComponent, "%i: %.*s", index, txDataOff - 1, txData);
                txDataOff = 0;
            }
            else if(txDataOff == sizeof(txData))
            {
                txDataOff = 0;
                logf(LogLevel::Warning, logComponent, "Dropping UART%i data", index);
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

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}
