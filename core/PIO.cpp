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
    //if(updateCallback && ~(hw.fstat & PIO_FSTAT_TXEMPTY_BITS))
    //    updateCallback(clock.getTime(), *this);

    if(!cycles)
        return;

    uint32_t clkdiv[NUM_PIO_STATE_MACHINES];
    uint32_t smCycles[NUM_PIO_STATE_MACHINES];

    for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
    {
        clkdiv[i] = hw.sm[i].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
        smCycles[i] = (cycles << 8) + clockFrac[i];
    }

    // start at last SM
    unsigned curSM = NUM_PIO_STATE_MACHINES - 1;
    auto fracCycles = smCycles[0];

    while(true)
    {
        // sync to next SM or end
        uint32_t target = curSM == NUM_PIO_STATE_MACHINES - 1 ? 0 : smCycles[curSM + 1];

        // calc max cycles to update
        auto maxSMCycles = (smCycles[curSM] - target) / clkdiv[curSM];

        // update
        uint32_t updated = 0;

        // skip if not enabled, otherwise run
        if(!(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + curSM))))
            updated = maxSMCycles;
        else
            updated = updateSM(curSM, maxSMCycles);

        // subtract executed cycles
        smCycles[curSM] -= updated * clkdiv[curSM];

        // go up 
        if(curSM > 0)
            curSM--;
        else if(smCycles[0] - target < clkdiv[0])
        {
            // first SM has reached target, go back down

            // add cycles
            // this may be a little off
            auto cycles = (fracCycles - smCycles[0]) >> 8;
            fracCycles -= cycles << 8;
            clock.addCycles(cycles);

            curSM++;
            auto nextTarget = curSM == NUM_PIO_STATE_MACHINES - 1 ? 0 : smCycles[curSM + 1];
            while(curSM < NUM_PIO_STATE_MACHINES && smCycles[curSM] - nextTarget < clkdiv[curSM])
            {
                curSM++;
                nextTarget = curSM >= NUM_PIO_STATE_MACHINES - 1 ? 0 : smCycles[curSM + 1];
            }

            // nothing left to update, done
            if(curSM == NUM_PIO_STATE_MACHINES)
                break;
        }
    }

    // save remaining fraction cycles
    for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
        clockFrac[i] = smCycles[i];
}

void PIO::setUpdateCallback(UpdateCallback cb)
{
    updateCallback = cb;
}

void PIO::setTXCallback(TXCallback cb)
{
    txCallback = cb;
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
            /*for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
            {
                if(txFifo[i].empty())
                    hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + i);
            }*/

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
                    {
                        logf(LogLevel::Debug, logComponent, "%i SM%i restarted", index, i);
                        regs[i].isr = 0;
                        regs[i].osc = regs[i].isc = 0;
                    }

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

        case PIO_SM0_INSTR_OFFSET:
        case PIO_SM1_INSTR_OFFSET:
        case PIO_SM2_INSTR_OFFSET:
        case PIO_SM3_INSTR_OFFSET:
        {
            update(time);

            int sm = (addr - PIO_SM0_INSTR_OFFSET) / (PIO_SM1_INSTR_OFFSET - PIO_SM0_INSTR_OFFSET);
            updateReg(hw.sm[sm].instr, data, atomic);

            logf(LogLevel::Debug, logComponent, "%i SM%i instr %04X", index, sm, hw.sm[sm].instr);

            // TODO: this does ignore the clkdiv, but this is sill maybe the wrong place...
            executeSMInstruction(sm, hw.sm[sm].instr);
            return;
        }

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

void PIO::dreqHandshake(uint64_t time, int dreq)
{
    // map DREQ to SM + rx/tx
    int sm = dreq;
    if(index == 1)
        sm -= (DREQ_PIO1_TX0 - DREQ_PIO0_TX0);

    bool isRX = sm >= DREQ_PIO0_RX0;
    
    sm -= isRX ? DREQ_PIO0_RX0 : DREQ_PIO0_TX0;

    if(isRX)
    {
        for(int i = 0; i < rxFifo->getCount(); i++)
            mem.getDMA().triggerDREQ(time, dreq);
    }
    else
    {
        for(int i = 0; i < 4 - txFifo->getCount(); i++)
            mem.getDMA().triggerDREQ(time, dreq);
    }
}

int PIO::getDREQNum(int sm, bool isTx) const
{
    return index * (DREQ_PIO1_TX0 - DREQ_PIO0_TX0) + (isTx ? 0 : DREQ_PIO0_RX0 - DREQ_PIO0_TX0) + sm;
}

unsigned PIO::updateSM(int sm, unsigned maxCycles)
{
    // TODO: EXEC latch, delays
    auto &pc = regs[sm].pc;

    unsigned cycles = maxCycles;
    while(cycles)
    {
        // run SM until something that might affect external state (PUSH, PULL, any output)
        // FIXME: output
        auto nextOp = hw.instr_mem[pc];
        if(cycles != maxCycles && (nextOp >> 13) == 4)
            break;

        if(executeSMInstruction(sm, nextOp))
        {
            // advance pc/wrap if we didn't jump or stall
            int wrapTop = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_TOP_BITS) >> PIO_SM0_EXECCTRL_WRAP_TOP_LSB;
            if(pc == wrapTop)
                pc = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB;
            else
                pc++;
        }

        cycles--;
    }
    return maxCycles - cycles;
}

bool PIO::executeSMInstruction(int sm, uint16_t op)
{
    // delay/side-set are common
    auto delaySide = (op >> 8) & 0x1F;

    int delay = 0;
    int sideSet = 0;

    if(delaySide)
    {
        auto sidesetCount = (hw.sm[sm].pinctrl & PIO_SM0_PINCTRL_SIDESET_COUNT_BITS) >> PIO_SM0_PINCTRL_SIDESET_COUNT_LSB;
        int delayBits = 5 - sidesetCount;

        delay = delaySide & ((1 << delayBits) - 1);
        sideSet = delaySide >> delayBits;

        static bool logDS = false;
        if(!logDS)
        {
            logf(LogLevel::NotImplemented, logComponent, "%i SM%i op %04X delay %i side-set %i", index, sm, op, delay, sideSet);
            logDS = true;
        }
    }

    auto &regs = this->regs[sm];

    auto getPushThreshold = [this](int sm)
    {
        int thresh = (hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_PUSH_THRESH_BITS) >> PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB;

        return thresh == 0 ? 32 : thresh;
    };

    auto getPullThreshold = [this](int sm)
    {
        int thresh = (hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS) >> PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB;

        return thresh == 0 ? 32 : thresh;
    };

    switch(op >> 13)
    {
        case 0: // JMP
        {
            auto cond = (op >> 5) & 0x7;
            auto addr = op & 0x1F;

            bool condVal = false;

            switch(cond)
            {
                case 0: // always
                    condVal = true;
                    break;
                case 1: // !X
                    condVal = regs.x == 0;
                    break;
                case 2: // X--
                    condVal = regs.x != 0;
                    regs.x--;
                    break;
                case 3: // !Y
                    condVal = regs.y == 0;
                    break;
                case 4: // Y--
                    condVal = regs.y != 0;
                    regs.y--;
                    break;
                case 5: // X != Y
                    condVal = regs.x != regs.y;
                    break;
                case 6: // PIN
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i JMP PIN", index, sm);
                    break;
                case 7: // !OSRE
                {
                    condVal = regs.osc < getPullThreshold(sm);
                    break;
                }
            }

            if(condVal)
            {
                regs.pc = addr;
                return false;
            }

            return true;
        }

        case 3: // OUT
        {
            auto dest = (op >> 5) & 0x7;
            auto count = op & 0x1F;

            bool autopull = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS;
            if(autopull)
            {
                if(regs.osc >= getPullThreshold(sm))
                {
                    // stall if empty
                    if(txFifo[sm].empty())
                    {
                        hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
                        return false;
                    }

                    // pull
                    regs.osr = txFifo[sm].pop();
                    regs.osc = 0;
                    // FIXME: these times are too early (we haven't updated the clock yet)
                    mem.getDMA().triggerDREQ(clock.getTime(), getDREQNum(sm, true));
                    if(txCallback)
                        txCallback(clock.getTime(), *this, sm, regs.osr);
                }
            }

            // get and shift
            bool shiftRight = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS;

            uint32_t data = 0;

            if(shiftRight)
            {
                data = regs.osr & ((1 << count) - 1);
                regs.osr >>= count;
            }
            else
            {
                data = (regs.osr >> (31 - count)) & ((1 << count) - 1);
                regs.osr <<= count;
            }

            regs.osc += count;

            switch(dest)
            {
                case 0: // PINS
                    break;
                case 1: // X
                    regs.x = data;
                    break;
                case 2: // Y
                    regs.y = data;
                    break;
                case 3: // NULL
                    break;
                case 4: // PINDIRS
                    break;
                case 5: // PC
                    regs.pc = data;
                    return false;
                case 6: // ISR
                    regs.isr = data;
                    regs.osc = count;
                    break;
                case 7: // EXEC
                    return false; // stall, we didn't do it

            }

            return true;
        }

        case 4: // PUSH/PULL
        {
            bool pull = op & (1 << 7);
            bool block = op & (1 << 5);

            if(!pull) // PUSH
            {
                bool ifFull = op & (1 << 6);

                if(ifFull)
                {
                    // ignore if below threshold
                    if(regs.isc < getPushThreshold(sm))
                        return true;
                }

                // stall if FIFO full
                if(rxFifo[sm].full())
                {
                    hw.fdebug |= 1 << (PIO_FDEBUG_RXSTALL_LSB + sm);

                    if(!block)
                    {
                        // non-blocking drops the data
                        regs.isc = 0;
                        regs.isr = 0;
                    }

                    return !block; // stall SM if block is set, otherwise ignore push
                }

                // FIFO not full, push
                rxFifo[sm].push(regs.isr);
                regs.isc = 0;
                regs.isr = 0;
                mem.getDMA().triggerDREQ(clock.getTime(), getDREQNum(sm, false));
            }
            else // PULL
            {
                bool ifEmpty = op & (1 << 6);

                if(ifEmpty)
                {
                    // ignore if below threshold
                    if(regs.osc < getPullThreshold(sm))
                        return true;
                }

                // stall if FIFO empty
                if(txFifo[sm].empty())
                {
                    if(block)
                    {
                        hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
                        return false; // stall SM
                    }
                    else
                    {
                        // move X to OSR for non-blocking
                        regs.osc = 0;
                        regs.osr = regs.x;
                        return true;
                    }
                }

                // FIFO not empty, pop
                regs.osr = txFifo[sm].pop();
                regs.osc = 0;
                mem.getDMA().triggerDREQ(clock.getTime(), getDREQNum(sm, true));
                if(txCallback)
                    txCallback(clock.getTime(), *this, sm, regs.osr);
            }
            return true;
        }

        case 5: // MOV
        {
            auto src = op & 0x7;
            auto movOp = (op >> 3) & 0x3;
            auto dest = (op >> 5) & 0x7;

            uint32_t val = 0;

            switch(src)
            {
                case 0: // PINS
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i MOV x, PINS", index, sm);
                    break;
                case 1: // X
                    val = regs.x;
                    break;
                case 2: // Y
                    val = regs.y;
                    break;
                case 3: // NULL
                    val = 0;
                    break;
                case 5: // STATUS
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i MOV x, STATUS", index, sm);
                    break;
                case 6: // ISR
                    val = regs.isr;
                    break;
                case 7: // OSR
                    val = regs.osr;
                    break;
            }

            if(movOp == 1) // invert
                val = ~val;
            else if(movOp == 2) // reverse
                logf(LogLevel::NotImplemented, logComponent, "%i SM%i MOV reverse", index, sm);

            switch(dest)
            {
                case 0: // PINS
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i MOV PINS, x", index, sm);
                    break;
                case 1: // X
                    regs.x = val;
                    break;
                case 2: // Y
                    regs.y = val;
                    break;
                case 4: // EXEC
                    //delay ignored
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i MOV EXEC, x", index, sm);
                    break;
                case 5: // PC
                    regs.pc = val;
                    return false;
                case 6: // ISR
                    regs.isr = val;
                    regs.isc = 0;
                    break;
                case 7: // OSR
                    regs.osr = val;
                    regs.osc = 0;
                    break;
            }

            return true;
        }

        case 7: // SET
        {
            auto dest = (op >> 5) & 0x7;
            auto data = op & 0x1F;

            switch(dest)
            {
                case 0: // PINS
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i SET PINS", index, sm);
                    break;
                case 1: // X
                    regs.x = data;
                    break;
                case 2: // Y
                    regs.y = data;
                    break;
                case 4: // PINDIRS
                    logf(LogLevel::NotImplemented, logComponent, "%i SM%i SET PINDIRS", index, sm);
                    break;
                
                // the rest are reserved
            }

            return true;
        }

        default:
        {
            static uint8_t logMask = 0;
            auto maskBit = 1 << (op >> 13);
            if(!(logMask & maskBit))
            {
                logf(LogLevel::NotImplemented, logComponent, "%i SM%i op %04X", index, sm, op);
                logMask |= maskBit;
            }
        }
    }

    return true;
}
