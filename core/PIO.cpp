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

// high three bits
enum class PIOOpcode
{
    JMP = 0,
    WAIT,
    IN,
    OUT,
    PUSH,
    PULL = PUSH,
    MOV,
    IRQ,
    SET,
};

enum class JmpCond
{
    Always = 0,
    NotX,
    XDec,
    NotY,
    YDec,
    XNotEqY,
    Pin,
    OutNotEmpty,
};

enum class MovDest
{
    Pins = 0,
    X,
    Y,
    Exec = 4,
    PC,
    ISR,
    OSR,
};

enum class MovOp
{
    None = 0,
    Invert,
    Reverse
};

// "extended" encoding
static constexpr uint8_t extendedOp(PIOOpcode op, int low)
{
    return static_cast<uint8_t>(op) << 5 | low;
}

static constexpr uint8_t jmpOp(JmpCond cond)
{
    return extendedOp(PIOOpcode::JMP, static_cast<int>(cond) << 2);
}

static constexpr uint8_t inOp()
{
    return extendedOp(PIOOpcode::IN, 0);
}

static constexpr uint8_t outOp()
{
    return extendedOp(PIOOpcode::OUT, 0);
}

static constexpr uint8_t pushOp()
{
    return extendedOp(PIOOpcode::PUSH, 0);
}

static constexpr uint8_t pullOp()
{
    return extendedOp(PIOOpcode::PULL, 1 << 4);
}

static constexpr uint8_t movOp(MovDest dest, MovOp op)
{
    return extendedOp(PIOOpcode::MOV, static_cast<int>(dest) << 2 | static_cast<int>(op));
}

static constexpr uint8_t setOp()
{
    return extendedOp(PIOOpcode::SET, 0);
}

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

    speedHack = false;

    for(auto &c : speedHackCounter)
        c = 0;

    for(auto &c : clockFrac)
        c = 0;

    txStall = rxStall = 0;

    for(auto &c : cyclesSinceLastPull)
        c = 0;
    
    for(auto &c : minCyclesBetweenPulls)
        c = 0;

    for(auto &c : cyclesBetweenPulls)
        c = 0;

    failedAnalysisAttempts = 0;
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

        // update
        // skip if not enabled, otherwise run
        if(!(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + curSM))))
        {
            auto maxSMCycles = (target - smCycles[curSM]) / clkdiv[curSM];
            smCycles[curSM] += maxSMCycles * clkdiv[curSM];
        }
        else
            updateSM(curSM, target, smCycles[curSM]);

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
                int32_t nextTarget = curSM == NUM_PIO_STATE_MACHINES - 1 ? (cycles << 8) : smCycles[curSM + 1];
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

void PIO::setSpeedHackEnabled(bool enabled)
{
    speedHack = enabled;
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
                    {
                        logf(LogLevel::Debug, logComponent, "%i SM%i enabled CLKDIV %08X EXECCTRL %08X SHIFTCTRL %08X PINCTRL %08X", index, i, hw.sm[i].clkdiv, hw.sm[i].execctrl, hw.sm[i].shiftctrl, hw.sm[i].pinctrl);
                        analyseProgram(i);
                    }

                    if(hw.ctrl & (1 << (PIO_CTRL_SM_RESTART_LSB + i)))
                    {
                        logf(LogLevel::Debug, logComponent, "%i SM%i restarted", index, i);
                        regs[i].isr = 0;
                        regs[i].osc = regs[i].isc = 0;

                        speedHackCounter[i] = 0;

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
            for(unsigned i = 0; i < NUM_PIO_STATE_MACHINES; i++)
            {
                instrs[i][off] = decodeInstruction(hw.instr_mem[off], i);
            
                if(hw.ctrl & (1 << (PIO_CTRL_SM_ENABLE_LSB + i)))
                    analyseProgram(i, off);
            }
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
            if(updateReg(hw.sm[sm].shiftctrl, data, atomic))
            {
                for(unsigned i = 0; i < PIO_INSTRUCTION_COUNT; i++)
                    instrs[sm][i] = decodeInstruction(hw.instr_mem[i], sm);
            }
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
            executeSMInstruction(sm, decodeInstruction(hw.sm[sm].instr, sm), clock.getCyclesToTime(time));
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

PIO::Instruction PIO::decodeInstruction(uint16_t op, int sm)
{
    Instruction instr;
    instr.op = (op >> 8) & 0xE0; // spare bits at the bottom

    // decode delay/side-set
    // TODO: SIDE_EN
    auto delaySideSet = (op >> 8) & 0x1F;

    auto sidesetCount = (hw.sm[sm].pinctrl & PIO_SM0_PINCTRL_SIDESET_COUNT_BITS) >> PIO_SM0_PINCTRL_SIDESET_COUNT_LSB;
    int delayBits = 5 - sidesetCount;

    instr.delay = delaySideSet & ((1 << delayBits) - 1);
    instr.sideSet = delaySideSet >> delayBits;

    if(delaySideSet)
        logf(LogLevel::NotImplemented, logComponent, "%i SM%i op %04X delay %i side-set %i", index, sm, op, instr.delay, instr.sideSet);

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
            instr.params[0] = (op >> 5) & 7; // source/dest
            instr.params[1] = op & 0x1F; // count

            if(instr.params[1] == 0)
                instr.params[1] = 32;

            if(op >> 13 == 2 && instr.params[0] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i IN PINS", index);

            // also OUT PINS, but we should add that one first anyway...
            if(op >> 13 == 3 && instr.params[0] == 4)
                logf(LogLevel::NotImplemented, logComponent, "%i OUT PINDIRS", index);
            if(op >> 13 == 3 && instr.params[0] == 7)
                logf(LogLevel::NotImplemented, logComponent, "%i OUT EXEC", index);
            break;

        case 7: // SET
            instr.params[0] = (op >> 5) & 7; // dest
            instr.params[1] = op & 0x1F; // data

            if( instr.params[0] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i SET PINS", index);
            if(instr.params[0] == 4)
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
        {
            // recode dest and (mov) op into op
            instr.op |= (op >> 3) & 0x1F;

            auto dest = (op >> 5) & 7; // dest
            auto movOp = (op >> 3) & 3; // op
            instr.params[0] = op & 7; // source

            // MOV Y, Y / NOP
            if(dest == 2 && movOp == 0 && instr.params[0] == 2)
                instr.op |= 3; // map to something invalid

            if(dest == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV PINS, x", index);
            else if(dest == 4)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV EXEC, x", index);

            if(movOp == 2)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV reverse", index);

            if(instr.params[0] == 0)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV x, PINS", index);
            else if(instr.params[0] == 5)
                logf(LogLevel::NotImplemented, logComponent, "%i MOV x, STATUS", index);
            break;
        }

        case 6: // IRQ
            // TODO
            logf(LogLevel::NotImplemented, logComponent, "%i IRQ %04X", index, op);
            break;
    }

    return instr;
}

void PIO::analyseProgram(int sm, int modInstrIndex)
{
    if(failedAnalysisAttempts > 1000)
        return;

    bool autopull = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS;
    bool autopush = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;

    int pullThreshold = (hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS) >> PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB;
    if(pullThreshold == 0)
        pullThreshold = 32;

    int wrapTop = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_TOP_BITS) >> PIO_SM0_EXECCTRL_WRAP_TOP_LSB;
    int wrapBottom = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB;

    if(modInstrIndex != -1 && (modInstrIndex < wrapBottom || modInstrIndex > wrapTop))
        return;

    static const char *instrNames[]{"JMP", "WAIT", "IN", "OUT", "PUSH/PULL", "MOV", "IRQ", "SET"};

    bool unhandled = false;

    int outCount = 32;
    int totalOutBits = 0;
    int cycles = 0;
    int numPulls = 0;

    // regs
    uint32_t x = 0, y = 0;
    bool xSet = false, ySet = false;

    for(int i = wrapBottom; i <= wrapTop; i++)
    {
        if(autopull && outCount >= pullThreshold)
        {
            outCount = 0;
            numPulls++;
        }

        auto &instr = instrs[sm][i];
        switch(instr.op)
        {
            case jmpOp(JmpCond::XDec):
            {
                if(xSet)
                {
                    // we can follow this if X was set to a constant
                    if(x-- != 0)
                        i = instr.params[0] - 1;
                }
                else
                {
                    logf(LogLevel::Debug, logComponent, "%i SM%i JMP unknown X", index, sm);
                    unhandled = true;
                }
                break;
            }

            case jmpOp(JmpCond::YDec):
            {
                if(ySet)
                {
                    // we can follow this if Y was set to a constant
                    if(y-- != 0)
                        i = instr.params[0] - 1;
                }
                else
                {
                    logf(LogLevel::Debug, logComponent, "%i SM%i JMP unknown Y", index, sm);
                    unhandled = true;
                }
                break;
            }

            case jmpOp(JmpCond::OutNotEmpty):
            {
                if(outCount < pullThreshold)
                    i = instr.params[0] - 1;
                break;
            }

            case inOp():
            {
                if(autopush)
                {
                    // we don't really handle input
                    unhandled = true;
                    logf(LogLevel::Debug, logComponent, "%i SM%i IN/PUSH", index, sm);
                }

                // doesn't have much effect with no PUSHes
                
                break;
            }

            case outOp():
            {
                outCount += instr.params[1];
                totalOutBits += instr.params[1];
                if(outCount > 32)
                {
                    totalOutBits -= outCount - 32;
                    outCount = 32;
                }

                switch(instr.params[0])
                {
                    case 0: // PINS
                    case 4: // PINDIRS
                    case 3: // NULL
                        break;
                    case 1: // X
                        xSet = false;
                        break;
                    case 2: // Y
                        ySet = false;
                        break;
                    case 5: // PC
                    case 6: // ISR
                    case 7: // EXEC
                        logf(LogLevel::Debug, logComponent, "%i SM%i OUT %i", index, sm, instr.params[0]);
                        unhandled = true;
                        break;
                }
                break;
            }

            case pullOp():
                // ignore ifEmpty?
                if(!autopull || outCount > 0)
                {
                    numPulls++;
                    outCount = 0;
                }
                break;

            case movOp(MovDest::Pins, MovOp::None):
            case movOp(MovDest::Pins, MovOp::Invert):
            case movOp(MovDest::Pins, MovOp::Reverse):
                break;

            case movOp(MovDest::X, MovOp::None):
            case movOp(MovDest::X, MovOp::Invert):
            case movOp(MovDest::X, MovOp::Reverse):
                xSet = false; // unless src was X, Y or NULL
                break;

            case movOp(MovDest::OSR, MovOp::None):
                outCount = 0; // resets shift counter
                break;

            case 0xAB: // NOP
                break;

            case setOp():
            {
                auto data = instr.params[1];

                switch(instr.params[0])
                {
                    case 0: // PINS
                    case 4: // PINDIRS
                        break;
                    case 1: // X
                        x = data;
                        xSet = true;
                        break;
                    case 2: // Y
                        y = data;
                        ySet = true;
                        break;
                }
                break;
            }

            default:
                logf(LogLevel::Debug, logComponent, "%i SM%i %s (%02X)", index, sm, instrNames[instr.op >> 5], instr.op);
                unhandled = true;
        }

        cycles++;
    }
    
    auto oldCycles = cyclesBetweenPulls[sm];

    cyclesBetweenPulls[sm] = 0;

    if(unhandled)
    {
        failedAnalysisAttempts++;
        return;
    }

    // if we have autopull and a program that shifts out all the bits before the end, we end up with an extra pull
    if(autopull && totalOutBits % pullThreshold == 0)
        numPulls--;

    // attempt to figure out how frequently this program will pull
    if(numPulls == 1 && totalOutBits && (pullThreshold % totalOutBits) == 0) // outputs fraction of a word (serial output)
        cyclesBetweenPulls[sm] = cycles * (pullThreshold / totalOutBits);
    else if(totalOutBits > pullThreshold && numPulls == 1) // only pulls once and probably outputs the entire thing
        cyclesBetweenPulls[sm] = cycles;
    else if(totalOutBits / pullThreshold == numPulls && totalOutBits % pullThreshold == 0) // multiple words per iteration
        cyclesBetweenPulls[sm] = cycles / numPulls;

    if(cyclesBetweenPulls[sm] != oldCycles)
        logf(LogLevel::Debug, logComponent, "%i SM%i (wrap %i -> %i) out %i bits in %i cycles (%i pulls, %i between pulls)", index, sm, wrapTop, wrapBottom, totalOutBits, cycles, numPulls, cyclesBetweenPulls[sm]);

    if(speedHack)
        minCyclesBetweenPulls[sm] = cyclesBetweenPulls[sm];
}

int PIO::getDREQNum(int sm, bool isTx) const
{
    return index * (DREQ_PIO1_TX0 - DREQ_PIO0_TX0) + (isTx ? 0 : DREQ_PIO0_RX0 - DREQ_PIO0_TX0) + sm;
}

void PIO::updateSM(int sm, uint32_t target, int32_t &cycleOffset)
{
    auto clkdiv = hw.sm[sm].clkdiv >> PIO_SM0_CLKDIV_FRAC_LSB;

    if(txStall & (1 << sm))
    {
        // if we're stalled and the fifo is still empty, skip
        if(txFifo[sm].empty())
        {
            hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
            cycleOffset += (target - cycleOffset) / clkdiv * clkdiv;
            return;
        }

        // otherwise clear stall
        txStall &= ~(1 << sm);
    }

    // in "speed hack" mode, reduce PIO program to a periodic FIFO read
    if(speedHack && cyclesBetweenPulls[sm])
    {
        int numCycles = (target - cycleOffset) / clkdiv;
        while(numCycles)
        {
            int step = std::min(numCycles, cyclesBetweenPulls[sm] - speedHackCounter[sm]);
            numCycles -= step;
            cycleOffset += clkdiv * step;

            speedHackCounter[sm] += step;

            if(speedHackCounter[sm] == cyclesBetweenPulls[sm])
            {
                if(txFifo[sm].empty())
                {
                    txStall |= 1 << sm;
                    hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
                }
                else
                {
                    auto data = txFifo[sm].pop();
                    auto time = clock.getTimeToCycles(cycleOffset >> 8);
                    mem.getDMA().triggerDREQ(time, getDREQNum(sm, true));
                    if(txCallback)
                        txCallback(time, *this, sm, data);
                }
                speedHackCounter[sm] = 0;
            }
        }
        return;
    }

    // TODO: EXEC latch, delays
    auto &pc = regs[sm].pc;
    int wrapTop = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_TOP_BITS) >> PIO_SM0_EXECCTRL_WRAP_TOP_LSB;
    int wrapBottom = (hw.sm[sm].execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB;

    auto nextOp = instrs[sm] + pc;

    while(cycleOffset + clkdiv <= target)
    {
        cycleOffset += clkdiv;

        auto res = executeSMInstruction(sm, *nextOp, cycleOffset >> 8);
        if(res == ExecResult::Done)
        {
            // advance pc/wrap if we didn't jump or stall
            if(pc == wrapTop)
                pc = wrapBottom;
            else
                pc++;
        }
        // stop if we stalled (we're not going to un-stall)
        else if(res == ExecResult::Stalled)
            break;

        cyclesSinceLastPull[sm]++;

        // run SM until something that might affect external state (PUSH, PULL, any output)
        // FIXME: output other than OUT?
        nextOp = instrs[sm] + pc;
        if(sm != 0 && ((nextOp->op >> 5) == 4 || (nextOp->op >> 5) == 3))
            break;
    }
}

PIO::ExecResult PIO::executeSMInstruction(int sm, const Instruction &instr, uint32_t clockOffset)
{
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

    auto doPush = [this, clockOffset, &regs](int sm)
    {
        rxFifo[sm].push(regs.isr);
        regs.isc = 0;
        regs.isr = 0;

        auto time = clock.getTimeToCycles(clockOffset);
        mem.getDMA().triggerDREQ(time, getDREQNum(sm, false));
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

    auto stallRX = [this](int sm)
    {
        rxStall |= 1 << sm;
        hw.fdebug |= 1 << (PIO_FDEBUG_RXSTALL_LSB + sm);
        return ExecResult::Stalled;
    };

    auto stallTX = [this](int sm)
    {
        txStall |= 1 << sm;
        hw.fdebug |= 1 << (PIO_FDEBUG_TXSTALL_LSB + sm);
        return ExecResult::Stalled;
    };

    auto jump = [this, &regs](int sm, uint8_t addr)
    {
        regs.pc = addr;
                
        // clear stall if jump forced
        rxStall &= ~(1 << sm);
        txStall &= ~(1 << sm);
        return ExecResult::Jumped;
    };

    auto getMovSrc = [this, &regs](int src) -> uint32_t
    {
        switch(src)
        {
            case 0: // PINS
                break;
            case 1: // X
                return regs.x;
            case 2: // Y
                return regs.y;
            case 3: // NULL
                return 0;
            case 5: // STATUS
                break;
            case 6: // ISR
                return regs.isr;
            case 7: // OSR
                return regs.osr;
        }
        return 0;
    };

    switch(instr.op)
    {
        case jmpOp(JmpCond::Always):
            return jump(sm, instr.params[0]);
    
        case jmpOp(JmpCond::NotX):
            if(regs.x == 0)
                return jump(sm, instr.params[0]);
            break;

        case jmpOp(JmpCond::XDec):
            if(regs.x-- != 0)
                return jump(sm, instr.params[0]);
            break;

        case jmpOp(JmpCond::NotY):
            if(regs.y == 0)
                return jump(sm, instr.params[0]);
            break;

        case jmpOp(JmpCond::YDec):
            if(regs.y-- != 0)
                return jump(sm, instr.params[0]);
            break;

        case jmpOp(JmpCond::XNotEqY):
            if(regs.x != regs.y)
                return jump(sm, instr.params[0]);
            break;

        case jmpOp(JmpCond::Pin):
            break; // TODO

        case jmpOp(JmpCond::OutNotEmpty):
            if(regs.osc < getPullThreshold(sm))
                return jump(sm, instr.params[0]);
            break;

        case inOp():
        {
            int count = instr.params[1];

            uint32_t data = 0;

            switch(instr.params[0])
            {
                case 0: // PINS
                    break;
                case 1: // X
                    data = regs.x;
                    break;
                case 2: // Y
                    data = regs.y;
                    break;
                case 3: // NULL
                    data = 0;
                    break;
                case 6: // ISR
                    data = regs.isr;
                    break;
                case 7: // OSR
                    data = regs.osr;
                    break;
            }

            data &= (1 << count) - 1;

            // shift and insert
            bool shiftRight = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS;

            if(shiftRight)
                regs.isr = regs.isr >> count | data << (32 - count);
            else
                regs.isr = regs.isr << count | data;

            regs.isc += count;
            if(regs.isc > 32)
                regs.isc = 32;

            bool autopush = hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;
            if(autopush)
            {
                if(regs.isc >= getPushThreshold(sm))
                {
                    // stall if empty
                    if(rxFifo[sm].full())
                        return stallRX(sm);

                    doPush(sm);
                }
            }

            break;
        }

        case outOp():
        {
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
                    // TODO: this should stall, but also we should be auto-pulling at the end of instrs
                }
            }

            int count = instr.params[1];

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
                data = (regs.osr >> (32 - count)) & ((1 << count) - 1);
                regs.osr <<= count;
            }

            regs.osc += count;
            if(regs.osc > 32)
                regs.osc = 32;

            switch(instr.params[0])
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
                    return ExecResult::Jumped;
                case 6: // ISR
                    regs.isr = data;
                    regs.osc = count;
                    break;
                case 7: // EXEC
                    return ExecResult::Stalled; // stall, we didn't do it

            }

            break;
        }

        case pushOp():
        {
            bool ifFull = instr.params[0];

            if(ifFull)
            {
                // ignore if below threshold
                if(regs.isc < getPushThreshold(sm))
                    return ExecResult::Done;
            }

            // stall if FIFO full
            if(rxFifo[sm].full())
            {
                // need to do this even if non-blocking
                hw.fdebug |= 1 << (PIO_FDEBUG_RXSTALL_LSB + sm);

                bool block = instr.params[1];

                if(!block)
                {
                    // non-blocking drops the data
                    regs.isc = 0;
                    regs.isr = 0;
                }

                return block ? stallRX(sm) : ExecResult::Done; // stall SM if block is set, otherwise ignore push
            }

            // FIFO not full, push
            doPush(sm);

            break;
        }

        case pullOp():
        {
            bool ifEmpty = instr.params[0];

            if(ifEmpty)
            {
                // ignore if below threshold
                if(regs.osc < getPullThreshold(sm))
                    return ExecResult::Done;
            }
            else if(regs.osc == 0)
            {
                // PULL is a no-op if autopull is enabled and OSR is full
                if(hw.sm[sm].shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS)
                    return ExecResult::Done;
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
                    return ExecResult::Done;
                }
            }

            // FIFO not empty, pop
            doPull(sm);
            break;
        }

        //case movOp(MovDest::Pins, MovOp::None):
        //case movOp(MovDest::Pins, MovOp::Invert):
        case movOp(MovDest::X, MovOp::None):
            regs.x = getMovSrc(instr.params[0]);
            break;
        case movOp(MovDest::X, MovOp::Invert):
            regs.x = ~getMovSrc(instr.params[0]);
            break;
        case movOp(MovDest::Y, MovOp::None):
            regs.y = getMovSrc(instr.params[0]);
            break;
        case movOp(MovDest::Y, MovOp::Invert):
            regs.y = ~getMovSrc(instr.params[0]);
            break;
        //case movOp(MovDest::Exec, MovOp::None):
        //case movOp(MovDest::Exec, MovOp::Invert):
        case movOp(MovDest::PC, MovOp::None):
            regs.pc = getMovSrc(instr.params[0]);
            return ExecResult::Jumped;
        case movOp(MovDest::PC, MovOp::Invert):
            regs.pc = ~getMovSrc(instr.params[0]);
            return ExecResult::Jumped;
        case movOp(MovDest::ISR, MovOp::None):
            regs.isr = getMovSrc(instr.params[0]);
            regs.isc = 0;
            break;
        case movOp(MovDest::ISR, MovOp::Invert):
            regs.isr = ~getMovSrc(instr.params[0]);
            regs.isc = 0;
            break;
        case movOp(MovDest::OSR, MovOp::None):
            regs.osr = getMovSrc(instr.params[0]);
            regs.osc = 0;
            break;
        case movOp(MovDest::OSR, MovOp::Invert):
            regs.osr = ~getMovSrc(instr.params[0]);
            regs.osc = 0;
            break;

        case setOp():
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

            break;
        }
    }

    return ExecResult::Done;
}
