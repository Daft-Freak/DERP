#pragma once
#include <cstdint>

#include "MemoryBus.h"

class ARMv6MCore final
{
public:
    ARMv6MCore();

    void reset();

    void run(int ms);

    MemoryBus &getMem() {return mem;}

    uint32_t getCycleCount() const {return cycleCount;}

private:
    enum class Reg
    {
        R0 = 0,
        R1,
        R2,
        R3,
        R4,
        R5,
        R6,
        R7,
        // ARM mode/high
        R8,
        R9,
        R10,
        R11,
        R12,
        R13,
        R14,
        R15,

        // banked
        R8_fiq,
        R9_fiq,
        R10_fiq,
        R11_fiq,
        R12_fiq,
        R13_fiq,
        R14_fiq,

        R13_svc,
        R14_svc,

        R13_abt,
        R14_abt,

        R13_irq,
        R14_irq,

        R13_und,
        R14_und,

        // aliases
        SP = R13, // also banked aliases...
        LR = R14,
        PC = R15
    };

    enum Flags
    {
        // control
        Flag_T = (1 << 5), // thumb
        Flag_F = (1 << 6), // FIQ disable
        Flag_I = (1 << 7), // IRQ disable 

        // condition codes
        Flag_V = (1 << 28),
        Flag_C = (1 << 29),
        Flag_Z = (1 << 30),
        Flag_N = (1 << 31)
    };

    Reg mapReg(Reg r) const
    {
        int iReg = static_cast<int>(r);
        if(!regBankOffset || iReg < 8 || r == Reg::PC)
            return r;

        if(regBankOffset != 8/*FIQ*/ && iReg < 13)
            return r;

        return static_cast<Reg>(iReg + regBankOffset);
    }

    uint32_t reg(Reg r) const {return regs[static_cast<int>(mapReg(r))];}
    uint32_t &reg(Reg r) {return regs[static_cast<int>(mapReg(r))];}

    // THUMB, first 8 regs, also used when we don't want to map
    uint32_t loReg(Reg r) const {return regs[static_cast<int>(r)];}
    uint32_t &loReg(Reg r) {return regs[static_cast<int>(r)];}

    uint32_t &getSPSR()
    {
        switch(cpsr & 0x1F)
        {
            case 0x1F: // System
                return spsr[5]; // system/user modes don't have a SPSR reg, but seems this needs to do something
            case 0x11: // FIQ
                return spsr[0];
            case 0x12: // IRQ
                return spsr[3];
            case 0x13: // SVC
                return spsr[1];
            case 0x17: // ABT
                return spsr[2];
            case 0x1B: // UND
                return spsr[4];
        }

        assert(!"Bad CPSR mode!");
        return spsr[5]; // invalid mode!
    }

    void modeChanged() // possibly
    {
        switch(cpsr & 0x1F)
        {
            case 0x10: // User
            case 0x1F: // System
                regBankOffset = 0;
                break;
            case 0x11: // FIQ
                regBankOffset = static_cast<int>(Reg::R8_fiq) - static_cast<int>(Reg::R8);
                break;
            case 0x13: // SVC
                regBankOffset = static_cast<int>(Reg::R13_svc) - static_cast<int>(Reg::R13);
                break;
            case 0x17: // ABT
                regBankOffset = static_cast<int>(Reg::R13_abt) - static_cast<int>(Reg::R13);
                break;
            case 0x12: // IRQ
                regBankOffset = static_cast<int>(Reg::R13_irq) - static_cast<int>(Reg::R13);
                break;
            case 0x1B: // UND
                regBankOffset = static_cast<int>(Reg::R13_und) - static_cast<int>(Reg::R13);
                break;
            default:
                assert(!"Bad CPSR mode");
        }

        curSP = mapReg(Reg::SP);
        curLR = mapReg(Reg::LR);
    }

    uint8_t readMem8(uint32_t addr, int &cycles, bool sequential = false) const;
    uint32_t readMem16(uint32_t addr, int &cycles, bool sequential = false);
    uint16_t readMem16Aligned(uint32_t addr, int &cycles, bool sequential = false);
    uint32_t readMem32(uint32_t addr, int &cycles, bool sequential = false);
    uint32_t readMem32Aligned(uint32_t addr, int &cycles, bool sequential = false);
    void writeMem8(uint32_t addr, uint8_t data, int &cycles, bool sequential = false);
    void writeMem16(uint32_t addr, uint16_t data, int &cycles, bool sequential = false);
    void writeMem32(uint32_t addr, uint32_t data, int &cycles, bool sequential = false);

    int runCycles(int cycles);

    int executeTHUMBInstruction();

    int doTHUMB01MoveShifted(uint16_t opcode, uint32_t pc);
    int doTHUMB0102(uint16_t opcode, uint32_t pc);
    int doTHUMB03(uint16_t opcode, uint32_t pc);
    int doTHUMB040506(uint16_t opcode, uint32_t pc);
    int doTHUMB04ALU(uint16_t opcode, uint32_t pc);
    int doTHUMB05HiReg(uint16_t opcode, uint32_t pc);
    int doTHUMB06PCRelLoad(uint16_t opcode, uint32_t pc);
    int doTHUMB0708(uint16_t opcode, uint32_t pc);
    int doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t pc);
    int doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t pc);
    int doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t pc);
    int doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t pc);
    int doTHUMB12LoadAddr(uint16_t opcode, uint32_t pc);
    int doTHUMB1314(uint16_t opcode, uint32_t pc);
    int doTHUMB13SPOffset(uint16_t opcode, uint32_t pc);
    int doTHUMB14PushPop(uint16_t opcode, uint32_t pc);
    int doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t pc);
    int doTHUMB1617(uint16_t opcode, uint32_t pc);
    int doTHUMB18UncondBranch(uint16_t opcode, uint32_t pc);
    int doTHUMB19LongBranchLink(uint16_t opcode, uint32_t pc);

    void updateTHUMBPC(uint32_t pc);

    static const uint32_t clockSpeed = 16*1024*1024;
    static const uint32_t signBit = 0x80000000;

    // registers
    uint32_t regs[31]{};
    uint32_t cpsr;
    uint32_t spsr[6]; // fiq, svc, abt, irq, und

    Reg curSP = Reg::SP, curLR = Reg::LR;
    int regBankOffset = 0;

    const uint8_t *pcPtr = nullptr;
    int pcSCycles = 0, pcNCycles = 0;

    // pipeline
    uint32_t fetchOp = 0, decodeOp = 0;

    // internal state
    bool halted;

    uint32_t cycleCount = 0;
    int lastExtraCycles = 0; // used to keep runFrame in sync

    uint32_t nextUpdateCycle = 0; // next cycle where something needs updated

    MemoryBus mem;
};