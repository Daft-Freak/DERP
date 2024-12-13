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

    txStall = rxStall = 0;

    for(auto &c : cyclesSinceLastPull)
        c = 0;
    
    for(auto &c : minCyclesBetweenPulls)
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

    // skip fully disabled instance
    if(!(hw.ctrl & PIO_CTRL_SM_ENABLE_BITS))
    {
        for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
        {
            auto clkdiv = hw.sm[i].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
            clockFrac[i] = (clockFrac[i] + (cycles << 8)) % clkdiv;
        }
        clock.addCycles(cycles);
        return;
    }

    int32_t clkdiv[NUM_PIO_STATE_MACHINES];
    int32_t smCycles[NUM_PIO_STATE_MACHINES];

    for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
    {
        clkdiv[i] = hw.sm[i].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
        smCycles[i] = -clockFrac[i];
    }

    // start at last SM
    unsigned curSM = NUM_PIO_STATE_MACHINES - 1;

    while(true)
    {
        // sync to next SM or end
        int32_t target = curSM == NUM_PIO_STATE_MACHINES - 1 ? (cycles << 8) : smCycles[curSM + 1];

        // calc max cycles to update
        auto maxSMCycles = (target - smCycles[curSM]) / clkdiv[curSM];

        // update
        // skip if not enabled, otherwise run
        if(!(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + curSM))))
            smCycles[curSM] += maxSMCycles * clkdiv[curSM];
        else
            updateSM(curSM, maxSMCycles, smCycles[curSM]);

        // go up 
        if(curSM > 0)
            curSM--;
        else
        {
            if(target - smCycles[0] < clkdiv[0])
            {
                // first SM has reached target, go back down
                curSM++;

                // ... until we find one that hasn't reached its target
                auto nextTarget = curSM == NUM_PIO_STATE_MACHINES - 1 ? (cycles << 8) : smCycles[curSM + 1];
                while(curSM < NUM_PIO_STATE_MACHINES && nextTarget - smCycles[curSM] < clkdiv[curSM])
                {
                    curSM++;
                    nextTarget = curSM >= NUM_PIO_STATE_MACHINES - 1 ? (cycles << 8) : smCycles[curSM + 1];
                }

                // nothing left to update, done
                if(curSM == NUM_PIO_STATE_MACHINES)
                    break;
            }
        }
    }

    clock.addCycles(cycles);

    // save remaining fraction cycles
    for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
        clockFrac[i] = (cycles << 8) - smCycles[i];
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

                        cyclesSinceLastPull[i] = minCyclesBetweenPulls[i] = 0;
                        rxStall &= ~(1 << i);
                        txStall &= ~(1 << i);
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
            instrs[off] = decodeInstruction(hw.instr_mem[off]);
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
            executeSMInstruction(sm, decodeInstruction(hw.sm[sm].instr), clock.getCyclesToTime(time));
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

PIO::Instruction PIO::decodeInstruction(uint16_t op)
{
    Instruction instr;
    instr.op = (op >> 8) & 0xE0; // spare bits at the bottom

    // would like to decode this further but it depends on per-sm state...
    instr.delaySideSet = (op >> 8) & 0x1F;

    switch(op >> 13)
    {
        case 0: // JMP
        {
            // recode cond into opcode
            instr.op |= ((op >> 5) & 7) << 2;

            instr.params[0] = op & 0x1F; // addr

            if(((op >> 5) & 7) == 6)
                logf(LogLevel::NotImplemented, logComponent, "%i JMP PIN", index);
            break;
        }
        case 2: // IN
        case 3: // OUT
        case 7: // SET
            instr.params[0] = (op >> 5) & 7; // source/dest/dest
            instr.params[1] = op & 0x1F; // count/count/data

            if(op >> 13 == 2)
                logf(LogLevel::NotImplemented, logComponent, "%i IN %04X", index, op);

            // also OUT PINS, but we should add that one first anyway...
            if(op >> 13 == 3 && instr.params[0] == 4)
                logf(LogLevel::NotImplemented, logComponent, "%i OUT PINDIRS", index);
            if(op >> 13 == 3 && instr.params[0] == 7)
                logf(LogLevel::NotImplemented, logComponent, "%i OUT EXEC", index);

            if(op >> 13 == 7 && instr.params[0] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i SET PINS", index);
            if(op >> 13 == 7 && instr.params[0] == 4)
                logf(LogLevel::NotImplemented, logComponent, "%i SET PINDIRS", index);
            break;

        case 1: // WAIT
            // TODO
            logf(LogLevel::NotImplemented, logComponent, "%i WAIT %04X", index, op);
            break;

        case 4: // PUSH/PULL
            // recode push/pull into op
            instr.op |= ((op >> 7) & 1) << 4;
            instr.params[0] = (op >> 6) & 1; // if full/empty
            instr.params[1] = (op >> 5) & 1; // block
            break;

        case 5: // MOV
            instr.params[0] = (op >> 5) & 7; // dest
            instr.params[1] = (op >> 3) & 3; // op
            instr.params[2] = op & 7; // source

            if(instr.params[0] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV PINS, x", index);
            else if(instr.params[0] == 4)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV EXEC, x", index);

            if(instr.params[1] == 2)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV reverse", index);

            if(instr.params[2] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV x, PINS", index);
            else if(instr.params[2] == 5)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV x, STATUS", index);
            break;

        case 6: // IRQ
            // TODO
            logf(LogLevel::NotImplemented, logComponent, "%i IRQ %04X", index, op);
            break;
    }

    return instr;
}

int PIO::getDREQNum(int sm, bool isTx) const
{
    return index * (DREQ_PIO1_TX0 - DREQ_PIO0_TX0) + (isTx ? 0 : DREQ_PIO0_RX0 - DREQ_PIO0_TX0) + sm;
}

void PIO::updateSM(int sm, unsigned maxCycles, int32_t &cycleOffset)
{
    auto clkdiv = hw.sm[sm].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;
    if(txStall & (1 << sm))
    {
        // if we're stalled and the fifo is still empty, skip
        if(txFifo[sm].empty())
        {
            hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
            cycleOffset += maxCycles * clkdiv;
            return;
        }

        // otherwise clear stall
        txStall &= ~(1 << sm);
    }

    // TODO: EXEC latch, delays
    auto &pc = regs[sm].pc;
    int wrapTop = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_TOP_BITS) >> PIO_SM0_EXECCTRL_WRAP_TOP_LSB;
    int wrapBottom = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB;

    auto nextOp = instrs + pc;

    unsigned cycles = maxCycles;
    while(cycles)
    {
        cycles--;
        cycleOffset += clkdiv;

        if(executeSMInstruction(sm, *nextOp, cycleOffset >> 8))
        {
            // advance pc/wrap if we didn't jump or stall
            if(pc == wrapTop)
                pc = wrapBottom;
            else
                pc++;
        }

        // stop if we stalled (we're not going to un-stall)
        if((txStall | rxStall) & (1 << sm))
            break;
        else
            cyclesSinceLastPull[sm]++;

        // run SM until something that might affect external state (PUSH, PULL, any output)
        // FIXME: output other than OUT?
        nextOp = instrs + pc;
        if(sm != 0 && ((nextOp->op >> 5) == 4 || (nextOp->op >> 5) == 3))
            break;
    }
}

bool PIO::executeSMInstruction(int sm, const Instruction &instr, uint32_t clockOffset)
{
    // delay/side-set are common
    auto delaySide = instr.delaySideSet;

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
            logf(LogLevel::NotImplemented, logComponent, "%i SM%i op %02X delay %i side-set %i", index, sm, instr.op, delay, sideSet);
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

    auto doPull = [this, clockOffset, &regs](int sm)
    {
        regs.osr = txFifo[sm].pop();
        regs.osc = 0;

        auto time = clock.getTimeToCycles(clockOffset);
        mem.getDMA().triggerDREQ(time, getDREQNum(sm, true));
        if(txCallback)
            txCallback(time, *this, sm, regs.osr);

        // track time between pulls
        auto pullCycles = cyclesSinceLastPull[sm];
        cyclesSinceLastPull[sm] = 0;

        if(!minCyclesBetweenPulls[sm] || pullCycles < minCyclesBetweenPulls[sm])
            minCyclesBetweenPulls[sm] = pullCycles;
    };

    auto stallTX = [this](int sm)
    {
        txStall |= 1 << sm;
        hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
        return false;
    };

    auto jump = [this, &regs](int sm, uint8_t addr)
    {
        regs.pc = addr;
                
        // clear stall if jump forced
        rxStall &= ~(1 << sm);
        txStall &= ~(1 << sm);
        return false;
    };

    switch(instr.op)
    {
        case 0x00: // JMP (always)
            return jump(sm, instr.params[0]);
    
        case 0x04: // JMP !X
            if(regs.x == 0)
                return jump(sm, instr.params[0]);
            return true;

        case 0x08: // JMP X--
            if(regs.x-- != 0)
                return jump(sm, instr.params[0]);
            return true;

        case 0x0C: // JMP !Y
            if(regs.y == 0)
                return jump(sm, instr.params[0]);
            return true;

        case 0x10: // JMP Y--
            if(regs.y-- != 0)
                return jump(sm, instr.params[0]);
            return true;

        case 0x14: // JMP X != Y
            if(regs.x != regs.y)
                return jump(sm, instr.params[0]);
            return true;

        case 0x18: // JMP PIN
            return true; // TODO

        case 0x1C: // JMP !OSRE
            if(regs.osc < getPullThreshold(sm))
                return jump(sm, instr.params[0]);
            return true;

        case 0x60: // OUT
        {
            auto dest = instr.params[0];
            auto count = instr.params[1];

            bool autopull = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS;
            if(autopull)
            {
                if(regs.osc >= getPullThreshold(sm))
                {
                    // stall if empty
                    if(txFifo[sm].empty())
                        return stallTX(sm);

                    // pull
                    doPull(sm);
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

        case 0x80: // PUSH
        {
            bool ifFull = instr.params[0];

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

                bool block = instr.params[1];

                if(block)
                    rxStall |= 1 << sm;
                else
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
            mem.getDMA().triggerDREQ(clock.getTimeToCycles(clockOffset), getDREQNum(sm, false));

            return true;
        }

        case 0x90: // PULL
        {
            bool ifEmpty = instr.params[0];

            if(ifEmpty)
            {
                // ignore if below threshold
                if(regs.osc < getPullThreshold(sm))
                    return true;
            }

            // stall if FIFO empty
            if(txFifo[sm].empty())
            {
                bool block = instr.params[1];

                if(block)
                    return stallTX(sm);
                else
                {
                    // move X to OSR for non-blocking
                    regs.osc = 0;
                    regs.osr = regs.x;
                    return true;
                }
            }

            // FIFO not empty, pop
            doPull(sm);
            return true;
        }

        case 0xA0: // MOV
        {
            auto src = instr.params[2];
            auto movOp = instr.params[1];
            auto dest = instr.params[0];

            uint32_t val = 0;

            switch(src)
            {
                case 0: // PINS
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
            {}

            switch(dest)
            {
                case 0: // PINS
                    break;
                case 1: // X
                    regs.x = val;
                    break;
                case 2: // Y
                    regs.y = val;
                    break;
                case 4: // EXEC
                    //delay ignored
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

        case 0xE0: // SET
        {
            auto dest = instr.params[0];
            auto data = instr.params[1];

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
                case 4: // PINDIRS
                    break;
                
                // the rest are reserved
            }

            return true;
        }
    }

    return true;
}
