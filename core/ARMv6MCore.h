#pragma once
#include <cstdint>

#include "ClockTarget.h"
#include "MemoryBus.h"

class ARMv6MCore final
{
public:
    ARMv6MCore(MemoryBus &mem);

    void reset();

    unsigned int run(int ms);
    unsigned int update(uint64_t target);

    void setPendingIRQ(int n);
    void setEvent();

    uint32_t readReg(uint32_t addr);
    void writeReg(uint32_t addr, uint32_t data);

    MemoryBus &getMem() {return mem;}

    ClockTarget &getClock() {return clock;}

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

        // the other SP
        PSP,

        // aliases
        SP = R13,
        MSP = SP,
        LR = R14,
        PC = R15
    };

    enum Flags
    {
        // control
        Flag_T = (1 << 24), // thumb

        // condition codes
        Flag_V = (1 << 28),
        Flag_C = (1 << 29),
        Flag_Z = (1 << 30),
        Flag_N = (1 << 31)
    };

    Reg mapReg(Reg r) const
    {
        if(r == Reg::SP && (cpsr & 0x3F) == 0 && control & (1 << 1))
            return Reg::PSP;

        return r;
    }

    uint32_t reg(Reg r) const {return regs[static_cast<int>(mapReg(r))];}
    uint32_t &reg(Reg r) {return regs[static_cast<int>(mapReg(r))];}

    // THUMB, first 8 regs, also used when we don't want to map
    uint32_t loReg(Reg r) const {return regs[static_cast<int>(r)];}
    uint32_t &loReg(Reg r) {return regs[static_cast<int>(r)];}

    uint8_t readMem8(uint32_t addr, int &cycles, bool sequential = false);
    uint16_t readMem16(uint32_t addr, int &cycles, bool sequential = false);
    uint32_t readMem32(uint32_t addr, int &cycles, bool sequential = false);
    void writeMem8(uint32_t addr, uint8_t data, int &cycles, bool sequential = false);
    void writeMem16(uint32_t addr, uint16_t data, int &cycles, bool sequential = false);
    void writeMem32(uint32_t addr, uint32_t data, int &cycles, bool sequential = false);

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
    int doTHUMBMisc(uint16_t opcode, uint32_t pc);
    int doTHUMB13SPOffset(uint16_t opcode, uint32_t pc);
    int doTHUMB14PushPop(uint16_t opcode, uint32_t pc);
    int doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t pc);
    int doTHUMB1617(uint16_t opcode, uint32_t pc);
    int doTHUMB18UncondBranch(uint16_t opcode, uint32_t pc);
    int doTHUMB32BitInstruction(uint16_t opcode, uint32_t pc);

    void updateTHUMBPC(uint32_t pc);

    int handleException();
    int handleExceptionReturn(uint32_t excRet);
    int getExceptionPriority(int exception) const;
    void checkPendingExceptions();

    void updateSysTick(int sysCycles = 0);

    static const uint32_t signBit = 0x80000000;

    // registers
    uint32_t regs[17]{};
    uint32_t cpsr;
    uint32_t primask, control;

    Reg curSP = Reg::SP;

    const uint8_t *pcPtr = nullptr;
    int pcSCycles = 0, pcNCycles = 0;

    // pipeline
    uint32_t fetchOp = 0, decodeOp = 0;

    // internal state
    bool sleeping, eventFlag;

    // exceptions
    uint64_t exceptionPending, exceptionActive;
    bool needException = false;

    // "real" time for synchronisation/scheduling
    ClockTarget clock;

    uint32_t sysTickRegs[4]; // E010-E01C
    uint32_t nvicEnabled, nvicPriority[8];
    uint32_t scbRegs[10]; // ED00-ED24
    uint32_t mpuRegs[5]; // ED90-EDA0

    MemoryBus &mem;
};