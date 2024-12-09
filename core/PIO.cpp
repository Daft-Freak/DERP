#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/dreq.h"
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

    for(auto &c : clockFrac)
        c = 0;
}

void PIO::update(uint64_t target)
{
    auto cycles = clock.getCyclesToTime(target);

    // this is almost certainly a hack to work around timing issues
    if(!cycles && updateCallback && ~(hw.fstat & PIO_FSTAT_TXEMPTY_BITS))
        updateCallback(clock.getTime(), *this);

    while(cycles)
    {
        auto step = cycles;

        // find next SM to update
        for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
        {
            if(!(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + i))))
                continue;

            auto clkdiv = hw.sm[i].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;

            auto toUpdate = (clkdiv - clockFrac[i] + 255) >> 8;

            if(toUpdate < step)
                step = toUpdate;
        }

        // step each SM
        for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
        {
            auto smCycles = (step << 8) + clockFrac[i];

            auto clkdiv = hw.sm[i].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
            if(smCycles > clkdiv)
            {
                smCycles -= clkdiv;
                clockFrac[i] = smCycles & 0xFF;

                if(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + i)))
                {} // run
            }
        }

        if(updateCallback)
            updateCallback(clock.getTime(), *this);
        else
        {
            // fake progress
            for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
            {
                if(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + i)))
                {
                    if(!txFifo[i].empty())
                        txFifo[i].pop();
                }
            }
        }

        clock.addCycles(step);

        cycles -= step;
    }
}

void PIO::setUpdateCallback(UpdateCallback cb)
{
    updateCallback = cb;
}

uint64_t PIO::getNextInterruptTime(uint64_t target)
{
    if(!hw.inte0 || !hw.inte1)
        return target;

    return target; // TODO
}

uint32_t PIO::regRead(uint64_t time, uint32_t addr)
{
    switch(addr)
    {
        case PIO_CTRL_OFFSET:
            return hw.ctrl;

        case PIO_FSTAT_OFFSET:
            update(time);

            for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
                updateFifoStatus(i);

            return hw.fstat;
        case PIO_FDEBUG_OFFSET:
        {
            update(time);

            // HACK: set txstall if FIFO empty
            // FIXME: this is wrong but we're not running the program
            for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
            {
                if(txFifo[i].empty())
                    hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + i);
            }

            return hw.fdebug;
        }

        case PIO_RXF0_OFFSET:
        case PIO_RXF1_OFFSET:
        case PIO_RXF2_OFFSET:
        case PIO_RXF3_OFFSET:
        {
            update(time);

            int index = (addr - PIO_RXF0_OFFSET) / 4;
            if(rxFifo[index].empty())
                hw.fdebug |= 1 << (PIO_FDEBUG_RXUNDER_LSB + index);
            else
            {
                auto ret = rxFifo[index].pop();
                mem.getDMA().triggerDREQ(getDREQNum(index, false));
                return ret;
            }

            return ~0;
        }

        case PIO_IRQ_OFFSET:
            logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);
            return 1; // PicoVision hack (causes swd load to fail instead of hang)

        case PIO_SM0_CLKDIV_OFFSET:
        case PIO_SM1_CLKDIV_OFFSET:
        case PIO_SM2_CLKDIV_OFFSET:
        case PIO_SM3_CLKDIV_OFFSET:
        {
            int sm = (addr - PIO_SM0_CLKDIV_OFFSET) / (PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET);
            return hw.sm[sm].clkdiv;
        }

        case PIO_SM0_EXECCTRL_OFFSET:
        case PIO_SM1_EXECCTRL_OFFSET:
        case PIO_SM2_EXECCTRL_OFFSET:
        case PIO_SM3_EXECCTRL_OFFSET:
        {
            int sm = (addr - PIO_SM0_EXECCTRL_OFFSET) / (PIO_SM1_EXECCTRL_OFFSET - PIO_SM0_EXECCTRL_OFFSET);
            return hw.sm[sm].execctrl;
        }

        case PIO_SM0_SHIFTCTRL_OFFSET:
        case PIO_SM1_SHIFTCTRL_OFFSET:
        case PIO_SM2_SHIFTCTRL_OFFSET:
        case PIO_SM3_SHIFTCTRL_OFFSET:
        {
            int sm = (addr - PIO_SM0_SHIFTCTRL_OFFSET) / (PIO_SM1_SHIFTCTRL_OFFSET - PIO_SM0_SHIFTCTRL_OFFSET);
            return hw.sm[sm].shiftctrl;
        }

        case PIO_SM0_ADDR_OFFSET:
        {
            update(time);

            static int counter = 0;
            return counter++ & PIO_SM0_ADDR_BITS;
        }

        // INSTR

        case PIO_SM0_PINCTRL_OFFSET:
        case PIO_SM1_PINCTRL_OFFSET:
        case PIO_SM2_PINCTRL_OFFSET:
        case PIO_SM3_PINCTRL_OFFSET:
        {
            int sm = (addr - PIO_SM0_PINCTRL_OFFSET) / (PIO_SM1_PINCTRL_OFFSET - PIO_SM0_PINCTRL_OFFSET);
            return hw.sm[sm].pinctrl;
        }
    }

    logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);
    return 0;
}

void PIO::regWrite(uint64_t time, uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case PIO_CTRL_OFFSET:
        {
            update(time);

            auto oldVal = hw.ctrl;
            if(updateReg(hw.ctrl, data, atomic))
            {
                auto enabled = (oldVal ^ hw.ctrl) & hw.ctrl & PIO_CTRL_SM_ENABLE_BITS;
                for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
                {
                    if(enabled & (1 << (PIO_CTRL_SM_ENABLE_LSB + i)))
                        logf(LogLevel::Debug, logComponent, "%i SM%i enabled CLKDIV %08X EXECCTRL %08X SHIFTCTRL %08X PINCTRL %08X", index, i, hw.sm[i].clkdiv, hw.sm[i].execctrl, hw.sm[i].shiftctrl, hw.sm[i].pinctrl);

                    if(hw.ctrl & (1 << (PIO_CTRL_SM_RESTART_LSB + i)))
                        logf(LogLevel::Debug, logComponent, "%i SM%i restarted", index, i);

                    if(hw.ctrl & (1 << (PIO_CTRL_CLKDIV_RESTART_LSB + 1)))
                        clockFrac[i] = 0;
                }

                // only the enable bits are written
                hw.ctrl &= PIO_CTRL_SM_ENABLE_BITS;
            }
            return;
        }
        
        case PIO_FDEBUG_OFFSET:
        {
            update(time);

            if(atomic == 0)
            {
                hw.fdebug &= ~data;
                return;
            }
            
            break;
        }

        case PIO_TXF0_OFFSET:
        case PIO_TXF1_OFFSET:
        case PIO_TXF2_OFFSET:
        case PIO_TXF3_OFFSET:
        {
            int index = (addr - PIO_TXF0_OFFSET) / 4;

            if(txFifo[index].full())
            {
                update(time);

                // still full after syncing, so REALLY full
                if(txFifo[index].full())
                {
                    hw.fdebug |= 1 << (PIO_FDEBUG_TXOVER_LSB + index);
                    return;
                }
            }

            txFifo[index].push(data);

            if(!txFifo[index].full())
                mem.getDMA().triggerDREQ(getDREQNum(index, true));

            return;
        }

        case PIO_INSTR_MEM0_OFFSET:
        case PIO_INSTR_MEM1_OFFSET:
        case PIO_INSTR_MEM2_OFFSET:
        case PIO_INSTR_MEM3_OFFSET:
        case PIO_INSTR_MEM4_OFFSET:
        case PIO_INSTR_MEM5_OFFSET:
        case PIO_INSTR_MEM6_OFFSET:
        case PIO_INSTR_MEM7_OFFSET:
        case PIO_INSTR_MEM8_OFFSET:
        case PIO_INSTR_MEM9_OFFSET:
        case PIO_INSTR_MEM10_OFFSET:
        case PIO_INSTR_MEM11_OFFSET:
        case PIO_INSTR_MEM12_OFFSET:
        case PIO_INSTR_MEM13_OFFSET:
        case PIO_INSTR_MEM14_OFFSET:
        case PIO_INSTR_MEM15_OFFSET:
        case PIO_INSTR_MEM16_OFFSET:
        case PIO_INSTR_MEM17_OFFSET:
        case PIO_INSTR_MEM18_OFFSET:
        case PIO_INSTR_MEM19_OFFSET:
        case PIO_INSTR_MEM20_OFFSET:
        case PIO_INSTR_MEM21_OFFSET:
        case PIO_INSTR_MEM22_OFFSET:
        case PIO_INSTR_MEM23_OFFSET:
        case PIO_INSTR_MEM24_OFFSET:
        case PIO_INSTR_MEM25_OFFSET:
        case PIO_INSTR_MEM26_OFFSET:
        case PIO_INSTR_MEM27_OFFSET:
        case PIO_INSTR_MEM28_OFFSET:
        case PIO_INSTR_MEM29_OFFSET:
        case PIO_INSTR_MEM30_OFFSET:
        case PIO_INSTR_MEM31_OFFSET:
        {
            update(time);
            int off = (addr - PIO_INSTR_MEM0_OFFSET) / 4;
            updateReg(hw.instr_mem[off], data, atomic);
            return;
        }

        case PIO_SM0_CLKDIV_OFFSET:
        case PIO_SM1_CLKDIV_OFFSET:
        case PIO_SM2_CLKDIV_OFFSET:
        case PIO_SM3_CLKDIV_OFFSET:
        {
            update(time);

            int sm = (addr - PIO_SM0_CLKDIV_OFFSET) / (PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET);
            updateReg(hw.sm[sm].clkdiv, data, atomic);
            return;
        }

        case PIO_SM0_EXECCTRL_OFFSET:
        case PIO_SM1_EXECCTRL_OFFSET:
        case PIO_SM2_EXECCTRL_OFFSET:
        case PIO_SM3_EXECCTRL_OFFSET:
        {
            update(time);

            int sm = (addr - PIO_SM0_EXECCTRL_OFFSET) / (PIO_SM1_EXECCTRL_OFFSET - PIO_SM0_EXECCTRL_OFFSET);
            updateReg(hw.sm[sm].execctrl, data, atomic);
            return;
        }

        case PIO_SM0_SHIFTCTRL_OFFSET:
        case PIO_SM1_SHIFTCTRL_OFFSET:
        case PIO_SM2_SHIFTCTRL_OFFSET:
        case PIO_SM3_SHIFTCTRL_OFFSET:
        {
            update(time);

            int sm = (addr - PIO_SM0_SHIFTCTRL_OFFSET) / (PIO_SM1_SHIFTCTRL_OFFSET - PIO_SM0_SHIFTCTRL_OFFSET);
            updateReg(hw.sm[sm].shiftctrl, data, atomic);
            return;
        }

        // ADDR, INSTR

        case PIO_SM0_PINCTRL_OFFSET:
        case PIO_SM1_PINCTRL_OFFSET:
        case PIO_SM2_PINCTRL_OFFSET:
        case PIO_SM3_PINCTRL_OFFSET:
        {
            update(time);

            int sm = (addr - PIO_SM0_PINCTRL_OFFSET) / (PIO_SM1_PINCTRL_OFFSET - PIO_SM0_PINCTRL_OFFSET);
            updateReg(hw.sm[sm].pinctrl, data, atomic);
            return;
        }
    }

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}

void PIO::updateFifoStatus(int sm)
{
    // RX
    if(rxFifo[sm].full())
        hw.fstat |= 1 << (PIO_FSTAT_RXFULL_LSB + sm);
    else
        hw.fstat &= ~(1 << (PIO_FSTAT_RXFULL_LSB + sm));

    if(rxFifo[sm].empty())
        hw.fstat |= 1 << (PIO_FSTAT_RXEMPTY_LSB + sm);
    else
        hw.fstat &= ~(1 << (PIO_FSTAT_RXEMPTY_LSB + sm));

    // TX
    if(txFifo[sm].full())
        hw.fstat |= 1 << (PIO_FSTAT_TXFULL_LSB + sm);
    else
        hw.fstat &= ~(1 << (PIO_FSTAT_TXFULL_LSB + sm));

    if(txFifo[sm].empty())
        hw.fstat |= 1 << (PIO_FSTAT_TXEMPTY_LSB + sm);
    else
        hw.fstat &= ~(1 << (PIO_FSTAT_TXEMPTY_LSB + sm));
}

void PIO::dreqHandshake(int dreq)
{
    // map DREQ to SM + rx/tx
    int sm = dreq;
    if(index == 1)
        sm -= (DREQ_PIO1_TX0 - DREQ_PIO0_TX0);

    bool isRX = sm >= DREQ_PIO0_RX0;
    
    sm -= isRX ? DREQ_PIO0_RX0 : DREQ_PIO0_TX0;

    if((isRX && !rxFifo[dreq].empty()) || (!isRX && !txFifo[dreq].full()))
        mem.getDMA().triggerDREQ(dreq);
}

int PIO::getDREQNum(int sm, bool isTx) const
{
    return index * (DREQ_PIO1_TX0 - DREQ_PIO0_TX0) + (isTx ? 0 : DREQ_PIO0_RX0 - DREQ_PIO0_TX0) + sm;
}
