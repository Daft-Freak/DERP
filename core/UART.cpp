#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/uart.h"

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
    hw.ibrd = UART_UARTIBRD_RESET;
    hw.fbrd = UART_UARTFBRD_RESET;
    hw.lcr_h = UART_UARTLCR_H_RESET;
    hw.cr = UART_UARTCR_RESET;

    txDataOff = 0;
}

uint32_t UART::regRead(uint32_t addr)
{
    switch(addr)
    {
        case UART_UARTFR_OFFSET:
            return UART_UARTFR_RXFE_BITS | UART_UARTFR_TXFE_BITS; // FIFOs empty
        case UART_UARTIBRD_OFFSET:
            return hw.ibrd;
        case UART_UARTFBRD_OFFSET:
            return hw.fbrd;
        case UART_UARTLCR_H_OFFSET:
            return hw.lcr_h;
        case UART_UARTCR_OFFSET:
            return hw.cr;
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
        case UART_UARTDR_OFFSET:
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
        case UART_UARTIBRD_OFFSET:
            updateReg(hw.ibrd, data, atomic);
            return;
        case UART_UARTFBRD_OFFSET:
            updateReg(hw.fbrd, data, atomic);
            return;
        case UART_UARTLCR_H_OFFSET:
            updateReg(hw.lcr_h, data, atomic);
            return;
        case UART_UARTCR_OFFSET:
            updateReg(hw.cr, data, atomic);
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}
