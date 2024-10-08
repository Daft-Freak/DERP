#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib> //exit
#include <cstring>
#include <limits>
#include <utility>

#include "hardware/regs/m0plus.h"

#include "ARMv6MCore.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::ArmCore;

#ifdef _MSC_VER
#define __builtin_unreachable() __assume(false)
#endif

// FIXME: this still thinks it's an ARMv4T

ARMv6MCore::ARMv6MCore(MemoryBus &mem) : debugHalted(false), debuggerAttached(false), mem(mem)
{}

void ARMv6MCore::reset()
{
    for(auto &reg: regs)
        reg = 0;

    cpsr = Flag_T;
    primask = control = 0;
    curSP = Reg::MSP;

    pcPtr = nullptr;

    sleeping = false;
    eventFlag = false;

    clock.reset();

    exceptionActive = exceptionPending = 0;
    needException = false;

    sysTickCSR = M0PLUS_SYST_CSR_RESET;
    sysTickReload = M0PLUS_SYST_RVR_RESET;
    sysTickCurrent = M0PLUS_SYST_CVR_RESET;
    sysTickCalib = M0PLUS_SYST_CALIB_RESET;

    nvicEnabled = 0;
    for(auto &reg : nvicPriority)
        reg = 0;

    for(auto &reg : scbRegs)
        reg = 0;

    for(auto &reg : mpuRegs)
        reg = 0;

    // CPUID
    // TODO: maybe not harcoded M0+ if I ever reuse this...
    scbRegs[0] = M0PLUS_CPUID_RESET;
    // CCR
    scbRegs[5] = 0x3F8;

    // MPU_TYPE
    mpuRegs[0] = M0PLUS_MPU_TYPE_RESET;

    int cycles;
    reg(Reg::SP) = mem.read<uint32_t>(this, 0, cycles); // MSP
    updateTHUMBPC( mem.read<uint32_t>(this, 4, cycles) & ~ 1); // Reset vector
}

unsigned int ARMv6MCore::run(int ms)
{
    auto targetTime = clock.getTargetTime(ms);
    return update(targetTime);
}

unsigned int ARMv6MCore::update(uint64_t target)
{
    unsigned int cycles = 0;

    mem.calcNextInterruptTime();

    while(clock.getTime() < target)
    {
        if(debugHalted)
            break;

        uint32_t exec = 1;

        if(!sleeping)
        {
            if(debuggerAttached && breakpoints.find(loReg(Reg::PC) - 2) != breakpoints.end())
            {
                debugHalted = true;
                break;
            }

            // CPU
            exec = executeTHUMBInstruction();
        }

        // loop until not halted or DMA was triggered
        uint64_t curTime;
        do
        {
            // interrupts?

            if(!(primask & 1) && needException)
                exec += handleException();

            clock.addCycles(exec);
            cycles += exec;

            // update systick if using cpu clock
            uint32_t mask = M0PLUS_SYST_CSR_ENABLE_BITS | M0PLUS_SYST_CSR_CLKSOURCE_BITS;
            if((sysTickCSR & mask) == mask)
                updateSysTick(exec);

            curTime = clock.getTime();

            if(mem.getNextInterruptTime() <= curTime)
            {
                mem.peripheralUpdate(curTime, sleeping ? ~0u : nvicEnabled, this);
                mem.calcNextInterruptTime();
            }

            if(sleeping && curTime < target)
            {
                // skip ahead
                auto skipTarget = std::min(target, mem.getNextInterruptTime());
                exec = std::max(UINT32_C(1), clock.getCyclesToTime(skipTarget, true));
            }
        }
        while(sleeping && curTime < target);
    }

    return cycles;
}

void ARMv6MCore::setPendingIRQ(int n)
{
    exceptionPending |= 1ull << (n + 16);
    checkPendingExceptions();
}

void ARMv6MCore::setEvent()
{
    eventFlag = true;
    if(sleeping)
        sleeping = false;
}

void ARMv6MCore::fault(const char *desc)
{
    auto curException = cpsr & 0x3F;
    if(curException == 2 /*NMI*/ || curException == 3 /*HardFault*/)
    {
        logf(LogLevel::Error, logComponent, "Lockup (%s) in %s at ~%08X", desc, curException == 2 ? "NMI" : "HardFault", reg(Reg::PC));
        exit(1);
    }

    logf(LogLevel::Error, logComponent, "Fault (%s) at ~%08X", desc, reg(Reg::PC));

    exceptionPending |= 1ull << 3; // HardFault
    checkPendingExceptions();
}

uint32_t ARMv6MCore::readReg(uint32_t addr)
{
    switch(addr & 0xFFFFFFF)
    {
        case M0PLUS_SYST_CSR_OFFSET:
            updateSysTick();
            return sysTickCSR;
        case M0PLUS_SYST_RVR_OFFSET:
            updateSysTick();
            return sysTickReload;
        case M0PLUS_SYST_CVR_OFFSET:
            updateSysTick();
            return sysTickCurrent;
        case M0PLUS_SYST_CALIB_OFFSET:
            updateSysTick();
            return sysTickCalib;
        
        case M0PLUS_NVIC_ISER_OFFSET:
        case M0PLUS_NVIC_ICER_OFFSET:
            return nvicEnabled;
        case M0PLUS_NVIC_ISPR_OFFSET:
        case M0PLUS_NVIC_ICPR_OFFSET:
            return exceptionPending >> 16;
        case M0PLUS_NVIC_IPR0_OFFSET:
        case M0PLUS_NVIC_IPR1_OFFSET:
        case M0PLUS_NVIC_IPR2_OFFSET:
        case M0PLUS_NVIC_IPR3_OFFSET:
        case M0PLUS_NVIC_IPR4_OFFSET:
        case M0PLUS_NVIC_IPR5_OFFSET:
        case M0PLUS_NVIC_IPR6_OFFSET:
        case M0PLUS_NVIC_IPR7_OFFSET:
            return nvicPriority[(addr & 0xFF) / 4];

        case M0PLUS_CPUID_OFFSET:
        case M0PLUS_ICSR_OFFSET:
        case M0PLUS_VTOR_OFFSET:
        case M0PLUS_AIRCR_OFFSET:
        case M0PLUS_SCR_OFFSET:
        case M0PLUS_CCR_OFFSET:
        case M0PLUS_SHPR2_OFFSET:
        case M0PLUS_SHPR3_OFFSET:
        case M0PLUS_SHCSR_OFFSET:
            return scbRegs[(addr & 0xFF) / 4];

        case M0PLUS_MPU_TYPE_OFFSET:
        case M0PLUS_MPU_CTRL_OFFSET:
        case M0PLUS_MPU_RNR_OFFSET:
        case M0PLUS_MPU_RBAR_OFFSET:
        case M0PLUS_MPU_RASR_OFFSET:
            return mpuRegs[((addr & 0xFF) - 0x90) / 4];
    }

    logf(LogLevel::NotImplemented, logComponent,"CPUI R %08X", addr);
    return 0;
}

void ARMv6MCore::writeReg(uint32_t addr, uint32_t data)
{
    switch(addr & 0xFFFFFFF)
    {
        case M0PLUS_SYST_CSR_OFFSET:
            updateSysTick();
            sysTickCSR = data;
            return;
        case M0PLUS_SYST_RVR_OFFSET:
            updateSysTick();
            sysTickReload = data;
            return;
        case M0PLUS_SYST_CVR_OFFSET:
            updateSysTick();
            sysTickCurrent = sysTickReload;
            return;
        
        case M0PLUS_NVIC_ISER_OFFSET:
            nvicEnabled |= data;
            checkPendingExceptions();
            return;
        case M0PLUS_NVIC_ICER_OFFSET: // NVIC_ICER
            nvicEnabled &= ~data; //
            checkPendingExceptions();
            return;
        case M0PLUS_NVIC_ISPR_OFFSET:
            exceptionPending |= static_cast<uint64_t>(data) << 16;
            checkPendingExceptions();
            return;
        case M0PLUS_NVIC_ICPR_OFFSET:
            exceptionPending &= ~(static_cast<uint64_t>(data) << 16);
            checkPendingExceptions();
            return;
        case M0PLUS_NVIC_IPR0_OFFSET:
        case M0PLUS_NVIC_IPR1_OFFSET:
        case M0PLUS_NVIC_IPR2_OFFSET:
        case M0PLUS_NVIC_IPR3_OFFSET:
        case M0PLUS_NVIC_IPR4_OFFSET:
        case M0PLUS_NVIC_IPR5_OFFSET:
        case M0PLUS_NVIC_IPR6_OFFSET:
        case M0PLUS_NVIC_IPR7_OFFSET:
            nvicPriority[(addr & 0xFF) / 4] = data;
            return;

        //case M0PLUS_ICSR_OFFSET:
        case M0PLUS_VTOR_OFFSET:
        //case M0PLUS_AIRCR_OFFSET:
        case M0PLUS_SCR_OFFSET:
        case M0PLUS_SHPR2_OFFSET:
        case M0PLUS_SHPR3_OFFSET:
        case M0PLUS_SHCSR_OFFSET:
            scbRegs[(addr & 0xFF) / 4] = data;
            return;

        case M0PLUS_MPU_TYPE_OFFSET:
        case M0PLUS_MPU_CTRL_OFFSET:
        case M0PLUS_MPU_RNR_OFFSET:
        case M0PLUS_MPU_RBAR_OFFSET:
        case M0PLUS_MPU_RASR_OFFSET:
            mpuRegs[((addr & 0xFF) - 0x90) / 4] = data;
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "CPUI W %08X = %08X", addr, data);
}

uint8_t ARMv6MCore::readMem8(uint32_t addr, int &cycles, bool sequential)
{
    return mem.read<uint8_t>(this, addr, cycles);
}

uint16_t ARMv6MCore::readMem16(uint32_t addr, int &cycles, bool sequential)
{
    assert((addr & 1) == 0);

    return mem.read<uint16_t>(this, addr, cycles);
}


uint32_t ARMv6MCore::readMem32(uint32_t addr, int &cycles, bool sequential)
{
    assert((addr & 3) == 0);

    return mem.read<uint32_t>(this, addr, cycles);
}

void ARMv6MCore::writeMem8(uint32_t addr, uint8_t data, int &cycles, bool sequential)
{
    mem.write<uint8_t>(this, addr, data, cycles);
}

void ARMv6MCore::writeMem16(uint32_t addr, uint16_t data, int &cycles, bool sequential)
{
    mem.write<uint16_t>(this, addr, data, cycles);
}

void ARMv6MCore::writeMem32(uint32_t addr, uint32_t data, int &cycles, bool sequential)
{
    mem.write<uint32_t>(this, addr, data, cycles);
}

int ARMv6MCore::executeTHUMBInstruction()
{
    auto &pc = loReg(Reg::PC); // not a low reg, but not banked
    uint16_t opcode = decodeOp;

    decodeOp = fetchOp;

    pc += 2;
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(*this, thumbPCPtr, pc));
        fetchOp = *thumbPCPtr;
    }
    else
    {
        int tmp;
        fetchOp = mem.read<uint16_t>(this, pc, tmp);
    }

    switch(opcode >> 12)
    {
        case 0x0: // format 1
            return doTHUMB01MoveShifted(opcode, pc);
        case 0x1: // formats 1-2
            return doTHUMB0102(opcode, pc);
        case 0x2: // format 3, mov/cmp immediate
        case 0x3: // format 3, add/sub immediate
            return doTHUMB03(opcode, pc);
        case 0x4: // formats 4-6
            return doTHUMB040506(opcode, pc);
        case 0x5: // formats 7-8
            return doTHUMB0708(opcode, pc);
        case 0x6: // format 9, load/store with imm offset (words)
            return doTHUMB09LoadStoreWord(opcode, pc);
        case 0x7: // ... (bytes)
            return doTHUMB09LoadStoreByte(opcode, pc);
        case 0x8: // format 10, load/store halfword
            return doTHUMB10LoadStoreHalf(opcode, pc);
        case 0x9: // format 11, SP-relative load/store
            return doTHUMB11SPRelLoadStore(opcode, pc);
        case 0xA: // format 12, load address
            return doTHUMB12LoadAddr(opcode, pc);
        case 0xB: // formats 13-14
            return doTHUMBMisc(opcode, pc);
        case 0xC: // format 15, multiple load/store
            return doTHUMB15MultiLoadStore(opcode, pc);
        case 0xD: // formats 16-17
            return doTHUMB1617(opcode, pc);
        case 0xE: // format 18, unconditional branch
            return doTHUMB18UncondBranch(opcode, pc);
        case 0xF: // format 19, long branch with link
            return doTHUMB32BitInstruction(opcode, pc);
    }

    __builtin_unreachable();
}

int ARMv6MCore::doTHUMB01MoveShifted(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x1;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto offset = (opcode >> 6) & 0x1F;
    auto res = loReg(srcReg);

    uint32_t carry;
    switch(instOp)
    {
        case 0: // LSL
            if(offset != 0)
            {
                carry = res & (1 << (32 - offset)) ? Flag_C : 0;
                res <<= offset;
            }
            else
                carry = cpsr & Flag_C; // preserve
            break;
        case 1: // LSR
            if(!offset) offset = 32; // shift by 0 is really 32

            carry = res & (1 << (offset - 1)) ? Flag_C : 0;
            if(offset == 32)
                res = 0;
            else
                res >>= offset;
            break;
        default:
            assert(!"Invalid format 1 shift type");
    }

    loReg(dstReg) = res;

    cpsr = (cpsr & 0x1FFFFFFF)
         | (res & signBit ? Flag_N : 0)
         | (res == 0 ? Flag_Z : 0)
         | carry;

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB0102(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    if(instOp == 3) // format 2, add/sub
    {
        bool isImm = opcode & (1 << 10);
        bool isSub = opcode & (1 << 9);
        uint32_t op1 = loReg(srcReg), op2 = (opcode >> 6) & 7;

        uint32_t res;

        if(!isImm)
            op2 = loReg(static_cast<Reg>(op2));

        uint32_t carry, overflow;

        cpsr &= 0x0FFFFFFF;

        if(isSub)
        {
            res = op1 - op2;
            carry = !(res > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & (op1 ^ res)) & signBit;
        }
        else
        {
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = (~(op1 ^ op2) & (op1 ^ res)) & signBit;
        }

        loReg(dstReg) = res;

        cpsr |= (res & signBit ? Flag_N : 0)
             | (res == 0 ? Flag_Z : 0)
             | carry
             | (overflow >> 3);
    }
    else // format 1, move shifted register
    {
        auto offset = (opcode >> 6) & 0x1F;
        auto res = loReg(srcReg);

        assert(instOp == 2); // others are handled elsewhere

        uint32_t carry;

        if(!offset) offset = 32;

        auto sign = res & signBit;
        carry = res & (1 << (offset - 1)) ? Flag_C : 0;
        if(offset == 32)
            res = sign ? 0xFFFFFFFF : 0;
        else
            res = static_cast<int32_t>(res) >> offset;

        loReg(dstReg) = res;

        cpsr = (cpsr & 0x1FFFFFFF)
             | (res & signBit ? Flag_N : 0)
             | (res == 0 ? Flag_Z : 0)
             | carry;
    }

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB03(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 11) & 0x3;
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t offset = opcode & 0xFF;

    auto dst = loReg(dstReg);

    uint32_t res;
    uint32_t carry, overflow;

    switch(instOp)
    {
        case 0: // MOV
            loReg(dstReg) = offset;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (offset == 0 ? Flag_Z : 0); // N not possible
            break;
        case 1: // CMP
            res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit; // offset cannot be negative, simplifies overflow checks
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3); // overflow is either 0 or 0x80000000, shift it down
            break;
        case 2: // ADD
            loReg(dstReg) = res = dst + offset;
            carry = res < dst ? Flag_C : 0;
            overflow = (~dst & res) & signBit;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 3: // SUB
            loReg(dstReg) = res = dst - offset;
            carry = !(res > dst) ? Flag_C : 0;
            overflow = (dst & ~res) & signBit;
            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        default:
            __builtin_unreachable();
    }

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB040506(uint16_t opcode, uint32_t pc)
{
    if(opcode & (1 << 11)) // format 6, PC-relative load
        return doTHUMB06PCRelLoad(opcode, pc);
    else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
        return doTHUMB05HiReg(opcode, pc);
    else // format 4, alu
        return doTHUMB04ALU(opcode, pc);
}

int ARMv6MCore::doTHUMB04ALU(uint16_t opcode, uint32_t pc)
{
    auto instOp = (opcode >> 6) & 0xF;
    auto srcReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto op1 = loReg(dstReg);
    auto op2 = loReg(srcReg);

    uint32_t res;
    uint32_t carry, overflow; // preserved if logical op

    switch(instOp)
    {
        case 0x0: // AND
            loReg(dstReg) = res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x1: // EOR
            loReg(dstReg) = res = op1 ^ op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x2: // LSL
            carry = cpsr & Flag_C;

            op2 &= 0xFF;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & 1) : 0;
                carry = carry ? Flag_C : 0;
                loReg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (32 - op2)) ? Flag_C : 0;
                loReg(dstReg) = res = op1 << op2;
            }
            else
                loReg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        case 0x3: // LSR
            carry = cpsr & Flag_C;

            op2 &= 0xFF;

            if(op2 >= 32)
            {
                carry = op2 == 32 ? (op1 & (1 << 31)) : 0;
                carry = carry ? Flag_C : 0;
                loReg(dstReg) = res = 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                loReg(dstReg) = res = op1 >> op2;
            }
            else
                loReg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        case 0x4: // ASR
        {
            op2 &= 0xFF;

            carry = cpsr & Flag_C;
            auto sign = op1 & signBit;
            if(op2 >= 32)
            {
                carry = sign ? Flag_C : 0;
                loReg(dstReg) = res = sign ? 0xFFFFFFFF : 0;
            }
            else if(op2)
            {
                carry = op1 & (1 << (op2 - 1)) ? Flag_C : 0;
                res = static_cast<int32_t>(op1) >> op2;

                loReg(dstReg) = res;
            }
            else
                loReg(dstReg) = res = op1;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            break;
        }
        case 0x5: // ADC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            loReg(dstReg) = res = op1 + op2 + c;
            carry = res < op1 || (res == op1 && c) ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x6: // SBC
        {
            int c = (cpsr & Flag_C) ? 1 : 0;
            loReg(dstReg) = res = op1 - op2 + c - 1;
            carry = !(op2 > op1 || (op2 == op1 && !c)) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0x7: // ROR
        {
            carry = cpsr & Flag_C;
            int shift = op2 & 0x1F;

            loReg(dstReg) = res = (op1 >> shift) | (op1 << (32 - shift));

            if(op2 & 0xFF)
                carry = res & (1 << 31) ? Flag_C : 0;

            cpsr = (cpsr & ~(Flag_C | Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry;
            return pcSCycles + 1;
        }
        case 0x8: // TST
            res = op1 & op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0x9: // NEG
        {
            loReg(dstReg) = res = 0 - op2;
            carry = !(op2 > 0) ? Flag_C : 0; //?
            overflow = (op2 & signBit) & (res & signBit);
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        }
        case 0xA: // CMP
            res = op1 - op2;
            carry = !(op2 > op1) ? Flag_C : 0;
            overflow = ((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // different signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xB: // CMN
            res = op1 + op2;
            carry = res < op1 ? Flag_C : 0;
            overflow = ~((op1 ^ op2) & signBit) & ((op1 ^ res) & signBit); // same signs and sign changed
            cpsr = (cpsr & 0x0FFFFFFF) | (res & signBit) | (res == 0 ? Flag_Z : 0) | carry | (overflow >> 3);
            break;
        case 0xC: // ORR
            loReg(dstReg) = res = op1 | op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0xD: // MUL
        {
            // carry is meaningless, v is unaffected
            loReg(dstReg) = res = op1 * op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);

            break; // single-cycle multiply
        }
        case 0xE: // BIC
            loReg(dstReg) = res = op1 & ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
        case 0xF: // MVN
            loReg(dstReg) = res = ~op2;
            cpsr = (cpsr & ~(Flag_N | Flag_Z)) | (res & signBit) | (res == 0 ? Flag_Z : 0);
            break;
    }

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB05HiReg(uint16_t opcode, uint32_t pc)
{
    auto op = (opcode >> 8) & 3;
    bool h1 = opcode & (1 << 7);
    bool h2 = opcode & (1 << 6);

    auto srcReg = static_cast<Reg>(((opcode >> 3) & 7) + (h2 ? 8 : 0));
    auto dstReg = static_cast<Reg>((opcode & 7) + (h1 ? 8 : 0));

    auto src = reg(srcReg);

    if(srcReg == Reg::SP)
        src &= ~3;

    switch(op)
    {
        case 0: // ADD
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC((loReg(Reg::PC) + src) & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else if(dstReg == Reg::SP)
                reg(dstReg) = (reg(dstReg) + src) & ~3; // low bits of SP should be zero
            else
                reg(dstReg) += src;

            break;
        case 1: // CMP
        {
            auto dst = reg(dstReg);

            auto res = dst - src;
            bool carry = !(src > dst);

            cpsr = (cpsr & ~(Flag_N | Flag_Z | Flag_C | Flag_V))
                    | ((res & signBit) ? Flag_N : 0)
                    | (res == 0 ? Flag_Z : 0)
                    | (carry ? Flag_C : 0)
                    | (((dst ^ src) & signBit) && ((dst ^ res) & signBit) ? Flag_V : 0);
            break;
        }
        case 2: // MOV
        {
            if(dstReg == Reg::PC)
            {
                updateTHUMBPC(src & ~1);
                return pcSCycles * 2 + pcNCycles;
            }
            else if(dstReg == Reg::SP)
                reg(dstReg) = src & ~3; // low bits of SP should be zero
            else
                reg(dstReg) = src;

            break;
        }
        case 3: // BX/BLX
        {
            if(h1) // BLX
                loReg(Reg::LR) = (pc - 2) | 1; 

            assert(src & 1);
            int cycles = pcSCycles * 2 + pcNCycles;

            if(src >> 28 == 0xF)
                cycles += handleExceptionReturn(src);
            else
                updateTHUMBPC(src & ~1);

            return cycles;
        }

        default:
            assert(!"Invalid format 5 op!");
    }

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB06PCRelLoad(uint16_t opcode, uint32_t pc)
{
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t word = opcode & 0xFF;

    // pc + 4, bit 1 forced to 0
    int cycles = 0;
    loReg(dstReg) = readMem32((pc & ~2) + (word << 2), cycles);

    return cycles + mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB0708(uint16_t opcode, uint32_t pc)
{
    auto offReg = static_cast<Reg>((opcode >> 6) & 7);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + loReg(offReg);

    if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
    {
        bool hFlag = opcode & (1 << 11);
        bool signEx = opcode & (1 << 10);

        if(signEx)
        {
            if(hFlag && !(addr & 1)) // LDRSH, (misaligned gets treated as a byte!)
            {
                int cycles = 0;
                auto val = readMem16(addr, cycles);
                if(val & 0x8000)
                    loReg(dstReg) = val | 0xFFFF0000;
                else
                    loReg(dstReg) = val;

                return cycles + mem.fetchTiming(pc, pcSCycles);
            }
            else // LDRSB
            {
                int cycles = 0;
                auto val = readMem8(addr, cycles);
                if(val & 0x80)
                    loReg(dstReg) = val | 0xFFFFFF00;
                else
                    loReg(dstReg) = val;

                return cycles + mem.fetchTiming(pc, pcSCycles);
            }
        }
        else
        {
            if(hFlag) // LDRH
            {
                int cycles = 0;
                loReg(dstReg) = readMem16(addr, cycles);
                return cycles + mem.fetchTiming(pc, pcSCycles);
            }
            else // STRH
            {
                int cycles = 0;
                writeMem16(addr, loReg(dstReg), cycles);
                return cycles + mem.fetchTiming(pc, pcNCycles);
            }
        }
    }
    else // format 7, load/store with reg offset
    {
        bool isLoad = opcode & (1 << 11);
        bool isByte = opcode & (1 << 10);

        if(isLoad)
        {
            int cycles = 0;
            if(isByte) // LDRB
                loReg(dstReg) = readMem8(addr, cycles);
            else // LDR
                loReg(dstReg) = readMem32(addr, cycles);

            return cycles + mem.fetchTiming(pc, pcSCycles);
        }
        else
        {
            int cycles = 0;
            if(isByte) // STRB
                writeMem8(addr, loReg(dstReg), cycles);
            else // STR
                writeMem32(addr, loReg(dstReg), cycles);

            return cycles + mem.fetchTiming(pc, pcNCycles);
        }
    }
}

int ARMv6MCore::doTHUMB09LoadStoreWord(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + (offset << 2);
    if(isLoad) // LDR
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + mem.fetchTiming(pc, pcSCycles);
    }
    else // STR
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + mem.fetchTiming(pc, pcNCycles);
    }
}

int ARMv6MCore::doTHUMB09LoadStoreByte(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F);
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRB
    {
        int cycles = 0;
        loReg(dstReg) = readMem8(addr, cycles);
        return cycles + mem.fetchTiming(pc, pcSCycles);
    }
    else // STRB
    {
        int cycles = 0;
        writeMem8(addr, loReg(dstReg), cycles);
        return cycles + mem.fetchTiming(pc, pcNCycles);
    }
}

int ARMv6MCore::doTHUMB10LoadStoreHalf(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto offset = ((opcode >> 6) & 0x1F) << 1;
    auto baseReg = static_cast<Reg>((opcode >> 3) & 7);
    auto dstReg = static_cast<Reg>(opcode & 7);

    auto addr = loReg(baseReg) + offset;
    if(isLoad) // LDRH
    {
        int cycles = 0;
        loReg(dstReg) = readMem16(addr, cycles);
        return cycles + mem.fetchTiming(pc, pcSCycles);
    }
    else // STRH
    {
        int cycles = 0;
        writeMem16(addr, loReg(dstReg), cycles);
        return cycles + mem.fetchTiming(pc, pcNCycles);
    }
}

int ARMv6MCore::doTHUMB11SPRelLoadStore(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    auto addr = loReg(curSP) + word;

    if(isLoad)
    {
        int cycles = 0;
        loReg(dstReg) = readMem32(addr, cycles);
        return cycles + mem.fetchTiming(pc, pcSCycles);
    }
    else
    {
        int cycles = 0;
        writeMem32(addr, loReg(dstReg), cycles);
        return cycles + mem.fetchTiming(pc, pcNCycles);
    }
}

int ARMv6MCore::doTHUMB12LoadAddr(uint16_t opcode, uint32_t pc)
{
    bool isSP = opcode & (1 << 11);
    auto dstReg = static_cast<Reg>((opcode >> 8) & 7);
    auto word = (opcode & 0xFF) << 2;

    if(isSP)
        loReg(dstReg) = loReg(curSP) + word;
    else
        loReg(dstReg) = (pc & ~2) + word; // + 4, bit 1 forced to 0

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMBMisc(uint16_t opcode, uint32_t pc)
{
    switch((opcode >> 8) & 0xF)
    {
        case 0x0: // add/sub imm to SP
            return doTHUMB13SPOffset(opcode, pc);

        case 0x2:
        {
            auto src = loReg(static_cast<Reg>((opcode >> 3) & 7));
            auto dstReg = static_cast<Reg>(opcode & 7);

            switch((opcode >> 6) & 3)
            {
                case 0x0: // SXTH
                    loReg(dstReg) = (src & 0x8000) ? src | 0xFFFF0000 : src & 0xFFFF;
                    break;
                case 0x1: // SXTB
                    loReg(dstReg) = (src & 0x80) ? src | 0xFFFFFF00 : src & 0xFF;
                    break;

                case 0x2: // UXTH
                    loReg(dstReg) = src & 0xFFFF;
                    break;
                case 0x3: // UXTB
                    loReg(dstReg) = src & 0xFF;
                    break;
            }

            return pcSCycles;
        }

        case 0x4: // PUSH
        case 0x5:
            return doTHUMB14PushPop(opcode, pc);

        case 0x6: // CPS
        {
            assert((opcode & 0xFFEF) == 0xB662);
            bool isPrivileged = (cpsr & 0x3F) != 0 || !(control & (1 << 0));

            if(isPrivileged)
                primask = (opcode & (1 << 4)) ? 1 : 0;

            return pcSCycles;
        }

        case 0xA:
        {
            auto src = loReg(static_cast<Reg>((opcode >> 3) & 7));
            auto dstReg = static_cast<Reg>(opcode & 7);

            switch((opcode >> 6) & 3)
            {
                case 0x0: // REV
                    loReg(dstReg) = src >> 24 | src << 24 | ((src << 8) & 0xFF0000) | ((src >> 8) & 0xFF00);
                    break;
                case 0x1: // REV16
                    loReg(dstReg) = ((src >> 8) & 0x00FF00FF) | ((src << 8) & 0xFF00FF00);
                    break;

                case 0x2: 
                    logf(LogLevel::Error, logComponent, "Invalid opcode %04X @%08X", opcode, pc - 4);
                    fault("Undefined instruction");
                    break;

                case 0x3: // REVSH
                    loReg(dstReg) = ((src >> 8) & 0x00FF) | ((src << 8) & 0xFF00);
                    if(src & 0x80)
                        loReg(dstReg) |= 0xFFFF0000; // sign extend

                    break;
            }
            return pcSCycles;
        }

        case 0xC: // POP
        case 0xD:
            return doTHUMB14PushPop(opcode, pc);

        case 0xE: // BKPT
            if(debuggerAttached)
                debugHalted = true;
            else
                fault("Unhandled BKPT");
            return pcSCycles;

        case 0xF: // hints
        {
            auto opA = (opcode >> 4) & 0xF;
            auto opB = opcode & 0xF;
            if(opB == 0)
            {
                switch(opA)
                {
                    case 0: // NOP
                        return pcSCycles;

                    case 1: // YIELD
                        return pcSCycles;

                    case 2: // WFE
                        if(eventFlag)
                            eventFlag = false;
                        else
                            sleeping = true;

                        return pcSCycles * 2;
                    
                    case 3: // WFI
                        // TODO: a bit different
                        return pcSCycles * 2;

                    case 4: // SEV
                        mem.sendEvent(this);
                        return pcSCycles;
                }
            }
        }
    }

    logf(LogLevel::Error, logComponent, "Unhandled opcode %04X @%08X", opcode, pc - 4);
    fault("Undefined instruction");
    return pcSCycles;
}

int ARMv6MCore::doTHUMB13SPOffset(uint16_t opcode, uint32_t pc)
{
    bool isNeg = opcode & (1 << 7);
    int off = (opcode & 0x7F) << 2;

    if(isNeg)
        loReg(curSP) -= off;
    else
        loReg(curSP) += off;

    return mem.fetchTiming(pc, pcSCycles);
}

int ARMv6MCore::doTHUMB14PushPop(uint16_t opcode, uint32_t pc)
{
    // timings here are probably off

    bool isLoad = opcode & (1 << 11);
    bool pclr = opcode & (1 << 8); // store LR/load PC
    uint8_t regList = opcode & 0xFF;

    int numRegs = pclr ? 1 : 0;
    for(uint8_t t = regList; t; t >>= 1)
    {
        if(t & 1)
            numRegs++;
    }

    int cycles = 0;

    if(isLoad) // POP
    {
        auto addr = loReg(curSP);
        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));
        auto loadCycles = mem.getAccessCycles(addr, 4);

        loReg(curSP) = addr + numRegs * 4;

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
            {
                regs[i] = *ptr++;
                cycles += loadCycles;
            }
        }

        if(pclr)
        {
            auto newPC = *ptr++;
            if(newPC >> 28 == 0xF)
                cycles += handleExceptionReturn(newPC);
            else
                updateTHUMBPC(newPC & ~1); /*ignore thumb bit*/

            cycles += loadCycles; // TODO
        }

        return mem.iCycle(cycles) + mem.fetchTiming(pc, pcSCycles);
    }
    else // PUSH
    {
        auto addr = loReg(curSP) - numRegs * 4;
        loReg(curSP) = addr;

        auto ptr = reinterpret_cast<uint32_t *>(mem.mapAddress(addr & ~3));
        auto storeCycles = mem.getAccessCycles(addr, 4);

        int i = 0;
        for(; regList; regList >>= 1, i++)
        {
            if(regList & 1)
            {
                *ptr++ = regs[i];
                cycles += storeCycles;
            }
        }

        if(pclr)
        {
            *ptr++ = loReg(Reg::LR);
            cycles += storeCycles;
        }

        return mem.iCycle(cycles) +  mem.fetchTiming(pc, pcNCycles);
    }
}

int ARMv6MCore::doTHUMB15MultiLoadStore(uint16_t opcode, uint32_t pc)
{
    bool isLoad = opcode & (1 << 11);
    auto baseReg = static_cast<Reg>((opcode >> 8) & 7);
    uint8_t regList = opcode & 0xFF;

    auto addr = loReg(baseReg);

    if(addr & 3)
        fault("Misaligned address in LDM/STM");

    int cycles = 0;

    if(!regList)
    {
        // empty list loads/stores PC... even though it isn't usually possible here
        if(isLoad)
            updateTHUMBPC(readMem32(addr, cycles) & ~1);
        else
        {
            writeMem32(addr, reg(Reg::LR), cycles);
            reg(baseReg) = addr + 4;
        }

        return cycles;
    }

    auto endAddr = addr;
    for(uint8_t t = regList; t; t >>=1)
    {
        if(t & 1)
            endAddr += 4;
    }

    int i = 0;
    bool seq = false;

    bool baseInList = regList & (1 << static_cast<int>(baseReg));

    for(; regList; regList >>= 1, i++)
    {
        if(!(regList & 1))
            continue;

        if(isLoad)
            regs[i] = readMem32(addr, cycles, seq);
        else
            writeMem32(addr, regs[i], cycles, seq);

        seq = true;

        addr += 4;
    }

    // prevent overriding base for loads
    if(!isLoad || !baseInList)
        reg(baseReg) = endAddr;

    if(isLoad)
        cycles += mem.fetchTiming(pc, pcSCycles);
    else
        cycles += mem.fetchTiming(pc, pcNCycles);

    return cycles;
}

int ARMv6MCore::doTHUMB1617(uint16_t opcode, uint32_t pc)
{
    // format 16, conditional branch (+ SWI)
    auto cond = (opcode >> 8) & 0xF;

    int offset = static_cast<int8_t>(opcode & 0xFF);
    bool condVal = false;
    switch(cond)
    {
        case 0x0: // BEQ
            condVal = cpsr & Flag_Z;
            break;
        case 0x1: // BNE
            condVal = !(cpsr & Flag_Z);
            break;
        case 0x2: // BCS
            condVal = cpsr & Flag_C;
            break;
        case 0x3: // BCC
            condVal = !(cpsr & Flag_C);
            break;
        case 0x4: // BMI
            condVal = cpsr & Flag_N;
            break;
        case 0x5: // BPL
            condVal = !(cpsr & Flag_N);
            break;
        case 0x6: // BVS
            condVal = cpsr & Flag_V;
            break;
        case 0x7: // BVC
            condVal = !(cpsr & Flag_V);
            break;
        case 0x8: // BHI
            condVal = (cpsr & Flag_C) && !(cpsr & Flag_Z);
            break;
        case 0x9: // BLS
            condVal = !(cpsr & Flag_C) || (cpsr & Flag_Z);
            break;
        case 0xA: // BGE
            condVal = !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
            break;
        case 0xB: // BLT
            condVal = !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
            break;
        case 0xC: // BGT
            condVal = !(cpsr & Flag_Z) && !!(cpsr & Flag_N) == !!(cpsr & Flag_V);
            break;
        case 0xD: // BLE
            condVal = (cpsr & Flag_Z) || !!(cpsr & Flag_N) != !!(cpsr & Flag_V);
            break;

        // E undefined

        /*case 0xF: // format 17, SWI
        {
            auto ret = (pc - 2) & ~1;
            spsr[1/ *svc* /] = cpsr;

            cpsr = (cpsr & ~(0x1F | Flag_T)) | Flag_I | 0x13; //supervisor mode
            modeChanged();
            loReg(curLR) = ret;
            updateARMPC(8);

            return pcSCycles * 2 + pcNCycles;
        }*/

        default:
            assert(!"Invalid THUMB cond");
    }

    if(!condVal)
        return pcSCycles; // no extra cycles if branch not taken
    
    updateTHUMBPC(pc + offset * 2);

    return pcSCycles * 2 + pcNCycles;
}

int ARMv6MCore::doTHUMB18UncondBranch(uint16_t opcode, uint32_t pc)
{
    assert(!(opcode & (1 << 11))); // 32 bit op

    uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2

    updateTHUMBPC(pc + offset);

    return pcSCycles * 2 + pcNCycles; // 2S + 1N
}

int ARMv6MCore::doTHUMB32BitInstruction(uint16_t opcode, uint32_t pc)
{
    // fetch second half
    uint32_t opcode32 = opcode << 16 | decodeOp;

    decodeOp = fetchOp;

    pc += 2;
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(*this, thumbPCPtr, pc));
        fetchOp = *thumbPCPtr;
    }
    else
    {
        int tmp;
        fetchOp = mem.read<uint16_t>(this, pc, tmp);
    }

    loReg(Reg::PC) = pc;

    // decode
    assert((opcode32 & 0x18008000) == 0x10008000);

    auto op1 = (opcode32 >> 20) & 0x7F;
    auto op2 = (opcode32 >> 12) & 0x7;

    if((op2 & 0b101) == 0b101) // BL
    {
        auto imm11 = opcode32 & 0x7FF;
        auto imm10 = (opcode32 >> 16) & 0x3FF;

        auto s = opcode32 & (1 << 26);
        auto i1 = (opcode32 >> 13) & 1;
        auto i2 = (opcode32 >> 11) & 1;

        if(!s)
        {
            i1 ^= 1;
            i2 ^= 1;
        }

        uint32_t offset = imm11 << 1 | imm10 << 12 | i2 << 22 | i1 << 23;

        if(s)
            offset |= 0xFF000000; // sign extend

        loReg(Reg::LR) = (pc - 2) | 1; // magic switch to thumb bit...
        updateTHUMBPC((pc - 2) + offset);

        return pcNCycles + pcSCycles * 3;
    }

    assert((op2 & 0b101) == 0);

    switch(op1)
    {
        case 0x38: // MSR
        case 0x39:
        {
            auto srcReg = static_cast<Reg>((opcode32 >> 16) & 0xF);
            auto sysm = opcode32 & 0xFF;
            bool isPrivileged = (cpsr & 0x3F) != 0 || !(control & (1 << 0));

            if((sysm >> 3) == 0)
            {
                // APSR
                cpsr = (cpsr & ~0xF8000000) | (reg(srcReg) & 0xF8000000);
                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 1)
            {
                // write MSP/PSP
                if(isPrivileged)
                {
                    if(sysm == 8)
                        loReg(Reg::MSP) = reg(srcReg) & ~3;
                    else if(sysm == 9)
                        loReg(Reg::PSP) = reg(srcReg) & ~3;
                }
                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 2)
            {
                // PRIMASK/CONTROL
                if(isPrivileged)
                {
                    if(sysm == 0x10)
                        primask = reg(srcReg) & 1;
                    else if(sysm == 0x14 && (cpsr & 0x3F) == 0)
                        control = reg(srcReg) & 3;
                }
                return pcSCycles * 2 + 1;
            }

            break;
        }

        case 0x3B: // misc
        {
            auto op = (opcode32 >> 4) & 0xF;

            if(op == 0x4 || op == 0x5) // DSB/DMB
            {
                //do something?
                return pcSCycles * 2 + 1;
            }

            break;
        }

        case 0x3E: // MRS
        case 0x3F:
        {
            auto dstReg = static_cast<Reg>((opcode32 >> 8) & 0xF);
            auto sysm = opcode32 & 0xFF;

            if((sysm >> 3) == 0)
            {
                // xPSR
                uint32_t mask = 0;
                if(sysm & 1) // IPSR
                    mask |= 0x1FF;

                // if(sysm & 2) // T bit reads as 0 so do nothing

                if((sysm & 4)== 0) // APSR
                    mask |= 0xF8000000;

                reg(dstReg) = cpsr & mask;

                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 1)
            {
                // MSP/PSP
                if(sysm == 8)
                    reg(dstReg) = loReg(Reg::MSP);
                else if(sysm == 9)
                    reg(dstReg) = loReg(Reg::PSP);

                return pcSCycles * 2 + 1;
            }
            else if((sysm >> 3) == 2)
            {
                // PRIMASK/CONTROL
                if(sysm == 0x10)
                    reg(dstReg) = primask & 1;
                else if(sysm == 0x14)
                    reg(dstReg) = control & 3;

                return pcSCycles * 2 + 1;
            }
            break;
        }
    }

    logf(LogLevel::Error, logComponent, "Unhandled opcode %08X @%08X",opcode32, pc - 6);
    fault("Undefined instruction");
    return pcSCycles;
}

void ARMv6MCore::updateTHUMBPC(uint32_t pc)
{
    // called when PC is updated in THUMB mode (except for incrementing)
    assert(!(pc & 1));

    // TODO: RP2040 specific ranges
    if((pc >= 0x40000000 && pc < 0x60000000) || pc >= 0xA0000000)
    {
        fault("XN");
        return;
    }

    if(pcPtr && pc >> 24 == loReg(Reg::PC) >> 24)
    {
        // memory region didn't change, skip recaclculating ptr/cycles
        [[maybe_unused]] auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        assert(mem.verifyPointer(*this, thumbPCPtr, pc));
    }
    else
    {
        pcPtr = std::as_const(mem).mapAddress(pc); // force const mapAddress
        if(pcPtr)
            pcPtr -= pc;
        pcSCycles = pcNCycles = mem.getAccessCycles(pc, 2);
    }

    mem.updatePC(pc);

    // refill the pipeline
    if(pcPtr)
    {
        auto thumbPCPtr = reinterpret_cast<const uint16_t *>(pcPtr + pc);
        decodeOp = *thumbPCPtr++;
        fetchOp = *thumbPCPtr;
    }
    else
    {
        // TODO: either fix the optimisation or remove it, this is messy
        int tmp;
        decodeOp = mem.read<uint16_t>(this, pc, tmp);
        fetchOp = mem.read<uint16_t>(this, pc + 2, tmp);
    }

    loReg(Reg::PC) = pc + 2; // pointing at last fetch
}

int ARMv6MCore::handleException()
{
    // get cur priority
    int curException = cpsr & 0x3F;
    int curPrio = getExceptionPriority(curException);

    // find highest priority pending exception
    int newException = 0;
    int newPrio = 4;

    for(int i = 2; i < 48 && newPrio; i++)
    {
        // skip not pending
        if(!(exceptionPending & (1ull << i)))
            continue;

        // skip not enabled external interrupt
        if(i >= 16 && !(nvicEnabled & (1 << (i - 16))))
            continue;

        int prio = getExceptionPriority(i);

        if(prio < newPrio)
        {
            newPrio = prio;
            newException = i;
        }
    }

    // no higher priority exception
    if(newPrio >= curPrio && newException >= curException)
        return 0;

    // push to stack
    auto &sp = reg(Reg::SP);
    auto spAlign = sp & 4;
    sp = (sp - 0x20) & ~4;

    int cycles = 0;
    writeMem32(sp +  0, loReg(Reg::R0 ), cycles);
    writeMem32(sp +  4, loReg(Reg::R1 ), cycles, true);
    writeMem32(sp +  8, loReg(Reg::R2 ), cycles, true);
    writeMem32(sp + 12, loReg(Reg::R3 ), cycles, true);
    writeMem32(sp + 16, loReg(Reg::R12), cycles, true);
    writeMem32(sp + 20, loReg(Reg::LR ), cycles, true);

    writeMem32(sp + 24, loReg(Reg::PC) - 2, cycles, true);
    writeMem32(sp + 28, cpsr | spAlign << 7, cycles, true);

    if(cpsr & 0x3F) // in handler
        loReg(Reg::LR) = 0xFFFFFFF1;
    else if(control & (1 << 1)/*SPSEL*/)
        loReg(Reg::LR) = 0xFFFFFFFD;
    else
        loReg(Reg::LR) = 0xFFFFFFF9;

    // take exception
    cpsr = (cpsr & ~0x3F) | newException;

    exceptionActive |= 1ull << newException;
    exceptionPending &= ~(1ull << newException);
    needException = false;

    // set event/wake up
    eventFlag = true;
    if(sleeping)
        sleeping = false;

    auto vtor = scbRegs[2];
    auto addr = readMem32(vtor + newException * 4, cycles);

    assert(addr & 1);
    updateTHUMBPC(addr & ~1);

    return cycles + pcSCycles * 2 + pcNCycles;
}

int ARMv6MCore::handleExceptionReturn(uint32_t excRet)
{
    assert((excRet & 0xFFFFFF0) == 0xFFFFFF0);

    int exception = cpsr & 0x3F;
    auto &sp = loReg((excRet & 0xF) == 0xD ? Reg::PSP : Reg::MSP);

    exceptionActive &= ~(1ull << exception);

    // pop from stack
    int cycles = 0;
    loReg(Reg::R0)  = readMem32(sp +  0, cycles);
    loReg(Reg::R1)  = readMem32(sp +  4, cycles, true);
    loReg(Reg::R2)  = readMem32(sp +  8, cycles, true);
    loReg(Reg::R3)  = readMem32(sp + 12, cycles, true);
    loReg(Reg::R12) = readMem32(sp + 16, cycles, true);
    loReg(Reg::LR)  = readMem32(sp + 20, cycles, true);
    
    auto newPC = readMem32(sp + 24, cycles, true);
    auto newPSR = readMem32(sp + 28, cycles, true);

    sp = (sp + 0x20) | (newPSR & (1 << 9)) >> 7;

    cpsr = newPSR & 0xF100003F;

    // set event
    eventFlag = true;
    if(sleeping)
        sleeping = false;

    // TODO: sleep on exit

    updateTHUMBPC(newPC & ~1);

    checkPendingExceptions();

    return cycles; // caller should handle the branch
}

int ARMv6MCore::getExceptionPriority(int exception) const
{
    switch(exception)
    {
        case 0: // thread/no exception
            return 4;

        case 2: // NMI
            return -2;
        
        case 3: // HardFault
            return -1;

        case 11: // SVCall
            return scbRegs[7]/*SHPR2*/ >> M0PLUS_SHPR2_PRI_11_LSB;
        case 14: // PendSV
            return  (scbRegs[8]/*SHPR3*/ & M0PLUS_SHPR3_PRI_14_BITS) >> M0PLUS_SHPR3_PRI_14_LSB;
        case 15: // SysTick
            return scbRegs[8]/*SHPR3*/ >> M0PLUS_SHPR3_PRI_15_LSB;
        
        default:
            assert(exception >= 16);
            // external interrupt
            int shift = 6 + (exception & 3) * 8;
            return (nvicPriority[(exception - 16) / 4] >> shift) & 3;
    }
}

void ARMv6MCore::checkPendingExceptions()
{
    needException = false;

    // mask disabled
    uint64_t mask = nvicEnabled << 16 | 0xFFFF;
    if(!(exceptionPending & mask))
        return;

    // get cur priority
    int curException = cpsr & 0x3F;
    int curPrio = getExceptionPriority(curException);

    // find highest priority pending exception
    int newException = 0;
    int newPrio = 4;

    uint64_t maskedExceptions = exceptionPending & mask;

    for(int i = 2; i < 48 && newPrio; i++)
    {
        // skip not pending
        if(!(maskedExceptions & (1ull << i)))
            continue;

        int prio = getExceptionPriority(i);

        if(prio < newPrio)
        {
            newPrio = prio;
            newException = i;
        }
    }

    // higher priority exception
    if(newPrio < curPrio || newException < curException)
        needException = true;
}

void ARMv6MCore::updateSysTick(int sysCycles)
{
    if(!(sysTickCSR & M0PLUS_SYST_CSR_CLKSOURCE_BITS))
        return; // TODO: watchdog tick

    sysTickCurrent = (sysTickCurrent - sysCycles) & M0PLUS_SYST_CVR_CURRENT_BITS;

    if(!sysTickCurrent)
        sysTickCurrent = sysTickReload;
}
