#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/intctrl.h"

#include "PIO.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::PIO;

PIO::PIO(MemoryBus &mem, int index) : mem(mem), index(index)
{
}

void PIO::reset()
{
    hw.ctrl = PIO_CTRL_RESET;
    hw.fstat = PIO_FSTAT_RESET;
    hw.fdebug = PIO_FDEBUG_RESET;
    hw.flevel = PIO_FLEVEL_RESET;

    hw.irq = PIO_IRQ_RESET;

    hw.input_sync_bypass = PIO_INPUT_SYNC_BYPASS_RESET;

    hw.dbg_padout = PIO_DBG_PADOUT_RESET;
    hw.dbg_padoe = PIO_DBG_PADOE_RESET;
    hw.dbg_cfginfo = PIO_DBG_CFGINFO_RESET;

    for(auto &sm : hw.sm)
    {
        sm.clkdiv = PIO_SM0_CLKDIV_RESET;
        sm.execctrl = PIO_SM0_EXECCTRL_RESET;
        sm.shiftctrl = PIO_SM0_SHIFTCTRL_RESET;
        sm.addr = PIO_SM0_ADDR_RESET;
        // sm.instr = PIO_SM0_INSTR_RESET;
        sm.pinctrl = PIO_SM0_PINCTRL_RESET;
    }

    hw.intr = PIO_INTR_RESET;
    hw.inte0 = PIO_IRQ0_INTE_RESET;
    hw.intf0 = PIO_IRQ0_INTF_RESET;
    hw.inte1 = PIO_IRQ1_INTE_RESET;
    hw.intf1 = PIO_IRQ1_INTF_RESET;
}

void PIO::update(uint64_t target)
{
    auto cycles = clock.getCyclesToTime(target);

    while(cycles)
    {
        auto step = cycles;

        clock.addCycles(step);

        cycles -= step;
    }
}

uint64_t PIO::getNextInterruptTime(uint64_t target)
{
    if(!hw.inte0 || !hw.inte1)
        return target;

    return target; // TODO
}

uint32_t PIO::regRead(uint32_t addr)
{
    switch(addr)
    {
        case PIO_FSTAT_OFFSET:
            return PIO_FSTAT_TXEMPTY_BITS | PIO_FSTAT_RXEMPTY_BITS; // all FIFOs empty
        case PIO_FDEBUG_OFFSET:
            return PIO_FDEBUG_TXSTALL_BITS; // all TXSTALL
        case PIO_SM0_ADDR_OFFSET:
        {
            static int counter = 0;
            return counter++ & PIO_SM0_ADDR_BITS;
        }
    }

    logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);
    return 0;
}

void PIO::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case PIO_TXF0_OFFSET:
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}
