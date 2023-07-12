#include <cassert>
#include <cstdio>
#include <functional>
#include <variant>

#include "X86Target.h"
#include "X86Builder.h"

enum CheckValueFlags
{
    Value_Immediate = 1 << 0,
    Value_Memory    = 1 << 1,
};

using RMOperand = X86Builder::RMOperand;

// reg helpers
static const Reg64 cpuPtrReg = Reg64::R14;

// only an output at the end, use the same reg used for the cycle count
static const Reg32 pcReg32 = Reg32::EDI;
static const Reg16 pcReg16 = Reg16::DI;

#ifdef _WIN32
static const Reg64 argumentRegs64[]{Reg64::RCX, Reg64::RDX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::ECX, Reg32::EDX, Reg32::R8D, Reg32::R9D};
#else
static const Reg64 argumentRegs64[]{Reg64::RDI, Reg64::RSI, Reg64::RDX, Reg64::RCX, Reg64::R8, Reg64::R9};
static const Reg32 argumentRegs32[]{Reg32::EDI, Reg32::ESI, Reg32::EDX, Reg32::ECX, Reg32::R8D, Reg32::R9D};
#endif

static const Reg64 callSavedRegs[]{Reg64::RAX, Reg64::RCX, Reg64::RDX, Reg64::RDI, Reg64::RSI};

static bool isXHReg(Reg8 reg)
{
    return static_cast<int>(reg) >= 4 && static_cast<int>(reg) < 8;
}

// more shift helpers
static bool doRegImmShift32(X86Builder &builder, std::optional<Reg32> dst, std::variant<std::monostate, Reg8, uint8_t> src, std::function<void(X86Builder &, Reg32)> regOp, std::function<void(X86Builder &, Reg32, uint8_t)> immOp, std::function<void(X86Builder &)> preOp, bool rotate = false)
{
    if(!src.index() || !dst)
        return false;

    if(std::holds_alternative<uint8_t>(src))
    {
        if(preOp)
            preOp(builder);

        auto imm = std::get<uint8_t>(src);
        assert(imm <= 32);
        if(imm == 32)
        {
            immOp(builder, *dst, 31);
            imm = 1;
        }
        immOp(builder, *dst, imm);
    }
    else
    {
        auto srcReg = std::get<Reg8>(src);
        auto srcReg32 = static_cast<Reg32>(srcReg);

        auto patchCondBranch = [&builder](uint8_t *branchPtr, Condition cond)
        {
            if(!branchPtr)
                return;

            auto off = builder.getPtr() - branchPtr - 2;
            builder.patch(branchPtr, branchPtr + 2);
            builder.jcc(cond, off);
            builder.endPatch();
        };

        auto patchBranch = [&builder](uint8_t *branchPtr)
        {
            if(!branchPtr)
                return;

            auto off = builder.getPtr() - branchPtr - 2;
            builder.patch(branchPtr, branchPtr + 2);
            builder.jmp(off);
            builder.endPatch();
        };

        // special cases
        uint8_t *ltBranch, *eqBranch = nullptr;
        uint8_t *gtDoneBranch = nullptr, *eqDoneBranch;

        builder.cmp(srcReg, 32);

        ltBranch = builder.getPtr();
        builder.jcc(Condition::B, 0); // < 32

        // skip the > case for rotates
        // in this case the condition above is != 32
        if(!rotate)
        {
            eqBranch = builder.getPtr();
            builder.jcc(Condition::E, 0); // == 32

            // > 32
            // need to set to 0 and set appropriate flags
            if(preOp)
                preOp(builder);
            immOp(builder, *dst, 31);
            immOp(builder, *dst, 2);
            gtDoneBranch = builder.getPtr();
            builder.jmp(0);
        }

        // == 32
        patchCondBranch(eqBranch, Condition::E);
        if(preOp)
            preOp(builder);
        immOp(builder, *dst, 31);
        immOp(builder, *dst, 1);
        eqDoneBranch = builder.getPtr();
        builder.jmp(0);

        // < 32 (!= 32 for rotates)
        patchCondBranch(ltBranch, rotate ? Condition::NE : Condition::B);

        bool swap = srcReg != Reg8::CL;

        if(swap)
            builder.xchg(srcReg32, Reg32::ECX);

        if(preOp)
            preOp(builder);

        // if src/dst were the same, they're now both ECX
        if(srcReg32 == *dst)
            regOp(builder, Reg32::ECX);
        // if it was the dst swap the args around
        else if(dst == Reg32::ECX)
            regOp(builder, srcReg32);
        else
            regOp(builder, *dst);

        if(swap)
            builder.xchg(srcReg32, Reg32::ECX);

        patchBranch(gtDoneBranch);
        patchBranch(eqDoneBranch);
    }

    return true;
}

void X86Target::init(SourceInfo sourceInfo, void *cpuPtr)
{
    static const Reg32 regList[]
    {
        Reg32::EAX,
        Reg32::ECX,
        Reg32::EDX,
        Reg32::EBX,
        Reg32::EBP,
        Reg32::R12D,
        Reg32::R13D,
        Reg32::R15D,

        Reg32::R11D, // last resort temp
    };

    numSavedRegs = 4; // TODO: can skip RDI/RSI on windows

    // count regs
    int numRegs = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(!reg.alias)
            numRegs++;
    }

    // enable using R15 if needed
    saveR15 = numRegs > 8;

    // alloc registers
    unsigned int allocOff = 0;

    regAlloc.emplace(0, Reg32::R10D); // temp

    int i = 1;
    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it, i++)
    {
        if(it->type == SourceRegType::Flags)
        {
            flagsReg = i;
            flagsSize = it->size;
        }

        if(it->alias)
            continue;

        // TODO: make sure we allocate all temps
        if(allocOff == std::size(regList))
            continue;

        if(it->type != SourceRegType::Temp && regList[allocOff] == Reg32::R11D)
            continue;

        regAlloc.emplace(i, regList[allocOff]);

        allocOff++;
    }

    // map flags
    for(auto & f : flagMap)
        f = 0xFF;

    i = 0;
    for(auto it = sourceInfo.flags.begin(); it != sourceInfo.flags.end(); ++it, i++)
    {
        flagMap[static_cast<int>(it->type)] = i;
        flagsMask |= 1 << it->bit;
    }

    // allocate the flags register if it isn't an alias
    if(!sourceInfo.registers[flagsReg].alias && !regAlloc.count(flagsReg))
    {
        regAlloc.emplace(flagsReg, Reg32::ESI);
        numSavedRegs++; // need to save it
    }

    this->sourceInfo = std::move(sourceInfo);
    this->cpuPtr = cpuPtr;
}

bool X86Target::compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint32_t pc, GenBlockInfo &blockInfo)
{
    X86Builder builder(codePtr, codeBufEnd);

    auto startPC = pc;

    // state
    uint8_t *lastInstrCycleCheck = nullptr;
    std::map<uint32_t, uint8_t *> branchTargets;
    std::multimap<uint32_t, uint8_t *> forwardBranchesToPatch;
    int needCallRestore = 0;


    // cycle executed sync
    int cyclesThisInstr = 0;
    int delayedCyclesExecuted = 0;
    bool stackCycleCount = false; // cycle count stored to stack

    auto cycleExecuted = [this, &builder, &needCallRestore, &blockInfo, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount]()
    {
        assert(!stackCycleCount);

        cyclesThisInstr += sourceInfo.cycleMul;

        // delay/inline cycle updates if possible
        delayedCyclesExecuted += sourceInfo.cycleMul;
    };

    auto syncCyclesExecuted = [this, &builder, &delayedCyclesExecuted, &stackCycleCount]()
    {
        if(!delayedCyclesExecuted)
            return;

        assert(!stackCycleCount);

        assert(delayedCyclesExecuted < 127);
        auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);

        int8_t i8Cycles = delayedCyclesExecuted;

        // we don't update cyclesToRun here, do it after returning instead
        int offset = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - cpuPtrInt;
        builder.addD({cpuPtrReg, offset}, i8Cycles);

        delayedCyclesExecuted = 0;
    };

    auto writeCycleCountToStack = [this, &builder, &needCallRestore, &cyclesThisInstr, &delayedCyclesExecuted, &stackCycleCount](int &instrCycles)
    {
        if(!stackCycleCount)
        {
            assert(cyclesThisInstr == delayedCyclesExecuted);
            builder.mov({Reg64::RSP, needCallRestore * 8}, uint32_t(cyclesThisInstr + instrCycles));

            cyclesThisInstr = delayedCyclesExecuted = 0;
            instrCycles = 0;
            stackCycleCount = true;
        }
        else
            assert(!cyclesThisInstr && !delayedCyclesExecuted);
    };

    // load/store helpers
    auto setupMemAddr = [this, &blockInfo, &builder, &syncCyclesExecuted](std::variant<std::monostate, RMOperand, uint32_t> addr, int addrSize, uint8_t addrIndex)
    {
        if(!addr.index())
            return; // err should already be set

        if(std::holds_alternative<RMOperand>(addr))
        {
            if(!sourceInfo.shouldSyncForRegIndex || sourceInfo.shouldSyncForRegIndex(addrIndex, blockInfo))
                syncCyclesExecuted();

            auto addrReg = std::get<RMOperand>(addr);

            builder.mov(argumentRegs32[1], addrReg);
        }
        else
        {
            auto immAddr = std::get<uint32_t>(addr);
            assert(addrSize == 32);

            if(!sourceInfo.shouldSyncForAddress || sourceInfo.shouldSyncForAddress(immAddr))
                syncCyclesExecuted();

            builder.mov(argumentRegs32[1], immAddr);
        }
    };

    auto setFlags32 = [this, &builder](Reg32 dst, Reg32 carryFlagCopy, uint16_t flags, bool invCarry = false, uint8_t haveResFlags = 0xF0)
    {
        if(!(flags & GenOp_WriteFlags))
            return;
    
        auto f = mapReg32(flagsReg);

        // overflow
        if(writesFlag(flags, SourceFlagType::Overflow))
        {
            builder.jcc(Condition::NO, 6);
            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Overflow).bit);
            haveResFlags = false;
        }

        // carry
        auto cFlag = flagWriteMask(SourceFlagType::Carry);
        if(flags & cFlag)
        {
            if(!(haveResFlags & cFlag))
            {
                builder.shl(carryFlagCopy, getFlagInfo(SourceFlagType::Carry).bit);
                builder.or_(*f, carryFlagCopy);
            }
            else
            {
                builder.jcc(invCarry ? Condition::B : Condition::AE, 6);

                builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Carry).bit);
            }
            haveResFlags = 0;
        }

        // negative
        auto nFlag = flagWriteMask(SourceFlagType::Negative);
        if(flags & nFlag)
        {
            if(haveResFlags & nFlag)
                builder.jcc(Condition::NS, 6);
            else
            {
                // dst < 0
                builder.cmp(dst, int8_t(0));
                builder.jcc(Condition::GE, 6);
            }

            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Negative).bit);
            haveResFlags = 0;
        }

        // zero
        auto zFlag = flagWriteMask(SourceFlagType::Zero);
        if(flags & zFlag)
        {
            if(!(haveResFlags & zFlag))
                builder.cmp(dst, int8_t(0));

            builder.jcc(Condition::NE, 6);
            builder.or_(*f, 1u << getFlagInfo(SourceFlagType::Zero).bit);
            haveResFlags = 0;
        }
    };

    // do instructions
    int numInstructions = 0;
    uint8_t *opStartPtr = nullptr;
    uint8_t *lastImmLoadStart = nullptr, *lastImmLoadEnd = nullptr;
    bool newEmuOp = true;

    auto beginInstr = blockInfo.instructions.begin();
    auto endInstr = blockInfo.instructions.end();

    for(auto instIt = beginInstr; instIt != endInstr; ++instIt)
    {
        auto &instr = *instIt;

        // handle branch targets
        if(instr.flags & GenOp_BranchTarget)
        {
            // store for backwards jumps
            if(lastInstrCycleCheck)
                branchTargets.emplace(pc, lastInstrCycleCheck); // after adjusting cycle count but before the jump
            else
            {
                // this is the first instruction, so make a cycle check for the branch to go to
                builder.jmp(12);
                lastInstrCycleCheck = builder.getPtr();

                // if <= 0 exit
                builder.jcc(Condition::G, 10);
                builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
                builder.call(saveAndExitPtr - builder.getPtr());

                // TODO: this is the first instruction
                branchTargets.emplace(pc, lastInstrCycleCheck);
            }

            // patch forwards jumps
            // can't hit this for the first instruction, so the lastInstrCycleCheck will be valid
            auto jumps = forwardBranchesToPatch.equal_range(pc);
            for(auto it = jumps.first; it != jumps.second; ++it)
            {
                // overrite 4 byte disp
                auto off = lastInstrCycleCheck - (it->second + 5);

                it->second[1] = off;
                it->second[2] = off >> 8;
                it->second[3] = off >> 16;
                it->second[4] = off >> 24;
            }
            forwardBranchesToPatch.erase(jumps.first, jumps.second);
        }

        pc += instr.len;

        // update start pointer if the last op was the end of an emulated op
        if(newEmuOp)
            opStartPtr = builder.getPtr();

        // get cycles
        int instrCycles = instr.cycles;

        bool err = false;

        // helpers to load/store registers we didn't allocate
        auto loadExtraReg32 = [this, &builder](uint8_t index, Reg32 dst)
        {
            auto offset = sourceInfo.getRegOffset(cpuPtr, index);
            builder.mov(dst, {cpuPtrReg, offset});
        };

        auto storeExtraReg32 = [this, &builder](uint8_t index, std::variant<std::monostate, Reg32, uint32_t> src)
        {
            assert(src.index());

            auto offset = sourceInfo.getRegOffset(cpuPtr, index);

            if(std::holds_alternative<uint32_t>(src))
                builder.mov({cpuPtrReg, offset}, std::get<uint32_t>(src));
            else
                builder.mov({cpuPtrReg, offset}, std::get<Reg32>(src));
        };

        // validating wrappers
        auto checkReg = [this, &builder, &needCallRestore, &err, &instr](uint8_t index, int size, bool allowExtra = false)
        {
            // this one doesn't return the reg, just validates and makes sure it's available
            bool found = false;
            int saveIndex = -1;

            if(size == 8)
            {
                if(auto reg = mapReg8(index))
                {
                    found = true;
                    saveIndex = callSaveIndex(*reg);
                }
            }
            else
            {
                // 16/32-bit registers are the same as far as this validation is concerned
                if(auto reg = mapReg32(index))
                {
                    found = true;
                    saveIndex = callSaveIndex(static_cast<Reg16>(*reg));
                }
            }

            if(!found)
            {
                if(allowExtra)
                    return true;

                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else if(saveIndex != -1)
                callRestore(builder, needCallRestore, saveIndex);

            return found;
        };

        auto checkReg32 = [this, &builder, &needCallRestore, &err, &instr, &loadExtraReg32](uint8_t index, std::optional<Reg32> loadReg = {}, bool allowExtra = false)
        {
            auto reg = mapReg32(index);
            if(!reg)
            {
                // if 
                if(sourceInfo.getRegOffset)
                { 
                    // return nothing but don't fail if the caller has promised to handle it
                    if(allowExtra)
                        return reg;

                    // load extra reg if we have somewhere to put it
                    if(loadReg)
                    {
                        loadExtraReg32(index, *loadReg);
                        return loadReg;
                    }

                    // otherwise fail
                }

                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else
                callRestoreIfNeeded(builder, static_cast<Reg16>(*reg), needCallRestore);

            return reg;
        };

        auto checkReg8 = [this, &builder, &needCallRestore, &err, &instr](uint8_t index)
        {
            auto reg = mapReg8(index);
            if(!reg)
            {
                // TODO: opcode labels
                printf("unhandled reg %s in op %i\n", sourceInfo.registers[index].label, int(instr.opcode));
                err = true;
            }
            else
                callRestoreIfNeeded(builder, *reg, needCallRestore);

            return reg;
        };

        // immediate helpers
        auto peekLastImmLoad = [&instIt, &beginInstr]() -> std::optional<uint32_t>
        {
            if(instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::LoadImm)
                return (instIt - 1)->imm;

            return {};
        };
    
        auto getLastImmLoad = [&blockInfo, &builder, &instIt, &beginInstr, &lastImmLoadStart, &lastImmLoadEnd]() -> std::optional<uint32_t>
        {
            if(instIt != beginInstr && (instIt - 1)->opcode == GenOpcode::LoadImm)
            {
                // remove the load
                assert(lastImmLoadStart);
                builder.removeRange(lastImmLoadStart, lastImmLoadEnd);
                lastImmLoadStart = lastImmLoadEnd = nullptr;

                return (instIt - 1)->imm;
            }

            return {};
        };


        auto checkRegOrImm8 = [&checkReg8, &getLastImmLoad](uint8_t index) -> std::variant<std::monostate, Reg8, uint8_t>
        {
            if(index == 0)
            {
                auto imm = getLastImmLoad();
                if(imm)
                {
                    assert(!(*imm & 0xFFFFFF00));
                    return static_cast<uint8_t>(*imm);
                }
            }

            if(auto reg = checkReg8(index))
                return *reg;

            return {};
        };

        auto checkValue32 = [this, &checkReg32, &getLastImmLoad, &err](uint8_t index, int flags = 0, std::optional<Reg32> loadReg = {}) -> std::variant<std::monostate, RMOperand, uint32_t>
        {
            if(index == 0 && (flags & Value_Immediate))
            {
                auto imm = getLastImmLoad();
                if(imm)
                    return *imm;
            }

            if(auto reg = checkReg32(index, loadReg, flags & Value_Memory))
                return RMOperand{*reg};

            if(!err) // must be memory, checkReg didn't set an error
            {
                auto offset = sourceInfo.getRegOffset(cpuPtr, index);
                return RMOperand{cpuPtrReg, offset};
            }

            return {};
        };

        // error handling
        auto badRegSize = [&err, &instr](int size)
        {
            printf("unhandled reg size %i in op %i\n", size, int(instr.opcode));
            err = true;
        };

        // helpers to deal with restrictions
        auto checkSingleSource = [this, &builder, &err, &instr, &checkReg32](bool canSwapSrcs = false)
        {
            if(instr.src[0] != instr.dst[0])
            {
                if(instr.src[1] == instr.dst[0] && canSwapSrcs)
                    return true;

                if(!sourceInfo.registers[instr.dst[0]].aliasMask && !sourceInfo.registers[instr.src[0]].aliasMask)
                {
                    auto dst = checkReg32(instr.dst[0]);
                    if(dst && instr.src[1] == instr.dst[0] && instr.opcode != GenOpcode::Not)
                    {
                        // dest is the second source, save it and replace the source
                        assert(instr.dst[0]); // it's already a temp

                        auto tmp = mapReg32(0);

                        if(instr.src[0] == 0)
                        {
                            printf("unhandled src[0] != dst in op %i\n", int(instr.opcode));
                            err = true;
                        }
                        else
                        {
                            // save dest in temp and use as second source
                            builder.mov(*tmp, *dst);
                            instr.src[1] = 0;
                        }
                    }

                    // move src0 to dst
                    auto src = checkReg32(instr.src[0], Reg32::R8D);

                    if(src && dst)
                    {
                        builder.mov(*dst, *src);
                        return false;
                    }
                }

                printf("unhandled src[0] != dst in op %i\n", int(instr.opcode));
                err = true;
            }

            return false;
        };

        // preserve flags
        // needs to be after the reg helpers
        uint32_t preserveMask = 0;

        if((instr.flags & GenOp_WriteFlags))
        {
            preserveMask = 0;
            for(int i = 0; i < 4; i++)
            {
                if(instr.flags & (1 << i))
                    preserveMask |= 1 << sourceInfo.flags[i].bit;
            }

            // preserve/clear flags
            if(flagsSize == 32)
            {
                // preserve carry if op needs it
                bool needCarry = instr.opcode == GenOpcode::AddWithCarry
                              || instr.opcode == GenOpcode::SubtractWithCarry
                              || instr.opcode == GenOpcode::RotateLeftCarry
                              || instr.opcode == GenOpcode::RotateRightCarry;

                if(needCarry)
                    preserveMask |= 1 << getFlagInfo(SourceFlagType::Carry).bit;

                // also preserve non-flags bits
                if(auto f = checkReg32(flagsReg))
                    builder.and_(*f, preserveMask | (~flagsMask));
            }
        }

        switch(instr.opcode)
        {
            case GenOpcode::NOP:
                break;

            case GenOpcode::LoadImm:
                lastImmLoadStart = builder.getPtr();
                builder.mov(*mapReg32(0), instr.imm);
                lastImmLoadEnd = builder.getPtr();
                break;

            case GenOpcode::Move:
            {
                auto regSize = sourceInfo.registers[instr.dst[0]].size;

                if(regSize == 32)
                {
                    auto dst = checkValue32(instr.dst[0], Value_Memory);

                    bool dstIsMem = std::holds_alternative<RMOperand>(dst) && std::get<RMOperand>(dst).isMem();

                    auto src = checkValue32(instr.src[0], Value_Immediate | (dstIsMem ? 0 : Value_Memory), Reg32::R8D);
                    
                    if(src.index() && dst.index())
                    {
                        auto rmDst = std::get<RMOperand>(dst);

                        if(std::holds_alternative<uint32_t>(src))
                            builder.mov(rmDst, std::get<uint32_t>(src));
                        else
                        {
                            auto rmSrc = std::get<RMOperand>(src);
                            assert(!rmSrc.isMem() || !rmDst.isMem());
                            if(rmDst.isMem())
                                builder.mov(rmDst, rmSrc.getReg32());
                            else
                                builder.mov(rmDst.getReg32(), rmSrc);
                        }

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        assert(!rmDst.isMem() || !(instr.flags & GenOp_WriteFlags));
                        setFlags32(rmDst.getReg32(), {}, instr.flags, false, 0);
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            case GenOpcode::Load:
            case GenOpcode::Load2:
            case GenOpcode::Load4:
            {
                // TODO: store data width somewhere
                auto addrSize = sourceInfo.registers[instr.src[0]].size;

                if(addrSize == 32)
                {
                    auto dst = checkReg8(instr.dst[0]);
                    auto addr = checkValue32(instr.src[0], Value_Immediate | Value_Memory);

                    if(addr.index() && dst)
                    {
                        // zero-extend if not 8 bit dest (a temp)
                        bool zeroExtend = instr.opcode == GenOpcode::Load && sourceInfo.registers[instr.dst[0]].size != 8;

                        // unless we want to sign extend instead
                        if(instr.flags & GenOp_SignExtend)
                            zeroExtend = false;
                        
                        // maybe push
                        callSaveIfNeeded(builder, needCallRestore);

                        bool needCyclesSeq = sourceInfo.readMem8;

                        setupMemAddr(addr, addrSize, instr.src[0]);

                        uint8_t *func = nullptr;

                        if(instr.opcode == GenOpcode::Load)
                            func = readMem8Ptr;
                        else if(instr.opcode == GenOpcode::Load2)
                            func = readMem16Ptr;
                        else if(instr.opcode == GenOpcode::Load4)
                            func = readMem32Ptr;

                        if(!func)
                        {
                            printf("unhandled load\n");
                            err = true;
                        }

                        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr

                        if(needCyclesSeq)
                        {
                            // cycle count from stack
                            builder.lea(argumentRegs64[2], {Reg64::RSP, needCallRestore * 8});
                            writeCycleCountToStack(instrCycles);

                            builder.mov(argumentRegs32[3], instr.flags >> 8); // flags
                        }

                        builder.call(func - builder.getPtr()); // do call

                        if(instr.opcode == GenOpcode::Load)
                        {
                            // 8-bit dest
                            if(isCallSaved(*dst))
                            {
                                callRestore(builder, *dst, zeroExtend, instr.flags & GenOp_SignExtend);
                                needCallRestore = 0;
                            }
                            else if(instr.flags & GenOp_SignExtend)
                                builder.movsx(static_cast<Reg32>(*dst), Reg8::AL);
                            else if(zeroExtend)
                                builder.movzx(static_cast<Reg32>(*dst), Reg8::AL);
                            else
                                builder.mov(*dst, Reg8::AL);
                        }
                        else
                        {
                            // 16/32-bit dest
                            bool saved = isCallSaved(*dst);
                            if(saved) // restore everything except RAX
                                callRestoreIfNeeded(builder, static_cast<Reg16>(callSavedRegs[1]), needCallRestore);

                            auto dst32 = static_cast<Reg32>(static_cast<int>(*dst) & 0xF); // bit of a hack

                            if(instr.opcode == GenOpcode::Load2 && (instr.flags & GenOp_SignExtend))
                                builder.movsx(dst32, Reg16::AX);
                            else if(dst32 != Reg32::EAX)
                                builder.mov(dst32, Reg32::EAX);

                            if(dst32 == Reg32::EAX)
                            {
                                // discard old RAX
                                builder.pop(Reg64::R10);
                                needCallRestore = 0;
                            }
                        }
                    }
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Store:
            case GenOpcode::Store2:
            case GenOpcode::Store4:
            {
                auto addrSize = sourceInfo.registers[instr.src[0]].size;
                int dataSize = 8;
                if(instr.opcode == GenOpcode::Store2)
                    dataSize = 16;
                else if(instr.opcode == GenOpcode::Store4)
                    dataSize = 32;

                if(addrSize == 32)
                {
                    auto addr = checkValue32(instr.src[0], Value_Immediate | Value_Memory);

                    if(addr.index() && checkReg(instr.src[1], dataSize, true))
                    {
                        bool updateCycles = instr.flags & GenOp_UpdateCycles;
                        if(updateCycles)
                            callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);

                        callSaveIfNeeded(builder, needCallRestore);

                        bool needCyclesSeq = sourceInfo.writeMem8;

#ifndef _WIN32
                        // setup addr first (data writes DX)
                        setupMemAddr(addr, addrSize, instr.src[0]);
#endif
                        
                        if(instr.opcode == GenOpcode::Store)
                        {
                            // TODO: this is mostly checkRegOrImm8, without the check
                            std::variant<std::monostate, Reg8, uint8_t> data;

                            if(instr.src[1] == 0)
                            {
                                if(auto imm = getLastImmLoad())
                                {
                                    assert(!(*imm & 0xFFFFFF00));
                                    data = static_cast<uint8_t>(*imm);
                                }
                            }

                            if(!data.index())
                                data = *mapReg8(instr.src[1]);

                            if(std::holds_alternative<Reg8>(data))
                            {
#ifdef _WIN32
                                // argumentRegs[2] is R8, can't mov from xH to there
                                auto reg8 = std::get<Reg8>(data);
                                if(isXHReg(reg8))
                                {
                                    builder.mov(Reg8::AL, reg8);
                                    data = Reg8::AL;
                                }
#endif
                                builder.movzx(argumentRegs32[2], std::get<Reg8>(data));
                            }
                            else
                                builder.mov(argumentRegs32[2], std::get<uint8_t>(data));
                        }
                        else
                        {
                            auto data32 = mapReg32(instr.src[1]); // already checked

                            if(!data32)
                                loadExtraReg32(instr.src[1], argumentRegs32[2]);
                            else if(instr.opcode == GenOpcode::Store2)
                                builder.movzx(argumentRegs32[2], static_cast<Reg16>(*data32));
                            else
                                builder.mov(argumentRegs32[2], *data32);
                        }

#ifdef _WIN32
                        // setup addr after data (addr writes DX)
                        setupMemAddr(addr, addrSize, instr.src[0]);
#endif

                        if(needCyclesSeq)
                        {
                            // FIXME: win32

                            // cycle count from stack
                            builder.lea(argumentRegs64[3], {Reg64::RSP, needCallRestore * 8});
                            writeCycleCountToStack(instrCycles);

                            builder.mov(argumentRegs32[4], instr.flags >> 8);

                            if(updateCycles)
                                builder.mov(argumentRegs32[5], Reg32::EDI); // cycle count
                        }
                        else if(updateCycles)
                            builder.mov(argumentRegs32[3], Reg32::EDI); // cycle count

                        // select function to call
                        uint8_t *func = nullptr;

                        if(instr.opcode == GenOpcode::Store)
                            func = writeMem8Ptr;
                        else if(instr.opcode == GenOpcode::Store2)
                            func = writeMem16Ptr;
                        else if(instr.opcode == GenOpcode::Store4)
                            func = writeMem32Ptr;

                        if(!func)
                        {
                            printf("unhandled store\n");
                            err = true;
                        }

                        builder.mov(argumentRegs64[0], cpuPtrReg); // cpu/this ptr
                        builder.call(func - builder.getPtr()); // do call

                        if(updateCycles)
                        {
                            // just pop EDI... then overrwrite it
                            callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);
                            builder.mov(Reg32::EDI, Reg32::EAX);
                        }
                    }
                }
                else
                    badRegSize(addrSize);

                break;
            }

            case GenOpcode::Add:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto writtenFlags = instr.flags & GenOp_WriteFlags;

                    // add 0 can't set C/V to 1
                    auto lastImm = peekLastImmLoad();
                    if((instr.src[0] == 0 || instr.src[1] == 0) && lastImm && !*lastImm)
                        writtenFlags &= ~(flagWriteMask(SourceFlagType::Carry) | flagWriteMask(SourceFlagType::Overflow));

                    if(writtenFlags)
                        checkSingleSource();

                    auto src = checkValue32(instr.src[1], Value_Immediate, Reg32::R8D);
                    auto dst = checkValue32(instr.dst[0], Value_Memory);

                    bool done = false;

                    if(!writtenFlags && instr.src[0] != instr.dst[0])
                    {
                        // we don't need the flags, so use LEA to avoid moves
                        auto src0 = checkValue32(instr.src[0], Value_Immediate, Reg32::R9D);

                        if(src0.index() && src.index() && dst.index())
                        {
                            bool src0Imm = std::holds_alternative<uint32_t>(src0);
                            bool src1Imm =  std::holds_alternative<uint32_t>(src);
                            assert(!src0Imm || !src1Imm);

                            // if dst is mem dst = r9
                            auto rmDst = std::get<RMOperand>(dst);

                            // use temp and store
                            if(rmDst.isMem())
                                rmDst = RMOperand{Reg32::R9D};

                            if(src0Imm || src1Imm)
                            {
                                auto imm = std::get<uint32_t>(src0Imm ? src0 : src);
                                auto reg = std::get<RMOperand>(src0Imm ? src : src0).base;

                                if(!imm) // replace add 0 with mov
                                    builder.mov(rmDst, static_cast<Reg32>(reg));
                                else
                                    builder.lea(rmDst.getReg32(), {reg, static_cast<int>(imm)});
                            }
                            else
                            {
                                auto reg0 = std::get<RMOperand>(src0).base;
                                auto reg1 = std::get<RMOperand>(src).base;
                                builder.lea(rmDst.getReg32(), {reg0, reg1});
                            }

                            // write back extra reg
                            if(rmDst.getReg32() == Reg32::R9D)
                                storeExtraReg32(instr.dst[0], rmDst.getReg32());

                            done = true;
                        }
                    }

                    if(src.index() && dst.index() && !done)
                    {
                        auto rmDst = std::get<RMOperand>(dst);
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80)
                                builder.addD(rmDst, static_cast<int8_t>(imm));
                            else
                                builder.add(rmDst, imm);
                        }
                        else
                            builder.add(rmDst, std::get<RMOperand>(src).getReg32());

                        // copy C flag if we need it
                        if(writesFlag(writtenFlags, SourceFlagType::Carry) && writesFlag(writtenFlags, SourceFlagType::Overflow))
                            builder.setcc(Condition::B, Reg8::R11B);

                        // flags
                        assert(!writtenFlags || !rmDst.isMem());
                        setFlags32(rmDst.getReg32(), Reg32::R11D, writtenFlags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::AddWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D);

                    if(src && dst)
                    {
                        // carry in (and clear it)
                        builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);

                        builder.adc(*dst, *src);

                        // copy C flag if we need it
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::B, Reg8::R11B);
                        
                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::And:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                bool swapped = checkSingleSource(true);

                if(regSize == 32)
                {
                    auto src = checkValue32(instr.src[swapped ? 0 : 1], Value_Immediate, Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80 || imm >= 0xFFFFFF80)
                                builder.and_(*dst, static_cast<int8_t>(imm));
                            else 
                                builder.and_(*dst, imm);
                        }
                        else
                            builder.and_(*dst, std::get<RMOperand>(src).getReg32());

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Compare:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                if(regSize == 32)
                {
                    auto writtenFlags = instr.flags & GenOp_WriteFlags;

                    auto src = checkValue32(instr.src[1], Value_Immediate, Reg32::R8D);
                    auto dst = checkReg32(instr.src[0], Reg32::R9D);
                    if(src.index() && dst)
                    {
                        // move the dest and do a sub to make flags handling easier
                        // could simplify if only one flag is used?
                        builder.mov(Reg32::R11D, *dst);
                    
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);

                            if(imm == 0) // cmp 0 can't set V to 1 (TODO: also always sets C)
                                writtenFlags &= ~flagWriteMask(SourceFlagType::Overflow);

                            if(imm < 0x80)
                                builder.sub(Reg32::R11D, static_cast<int8_t>(imm));
                            else
                                builder.sub(Reg32::R11D, imm);
                        }
                        else
                            builder.sub(Reg32::R11D, std::get<RMOperand>(src).getReg32());

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(writtenFlags, SourceFlagType::Carry) && writesFlag(writtenFlags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R10B);

                        // flags
                        setFlags32(Reg32::R11D, Reg32::R10D, writtenFlags, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Multiply:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src && dst)
                    {
                        builder.imul(*dst, *src);
                    
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags, false, 0);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Or:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkValue32(instr.src[1], Value_Immediate);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            // use sign extended if imm < 0x80? 
                            builder.or_(*dst, imm);
                        }
                        else
                            builder.or_(*dst, std::get<RMOperand>(src).getReg32());
                    
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Subtract:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                bool swapped = checkSingleSource(regSize == 32);

                if(regSize == 32)
                {
                    auto src = checkValue32(instr.src[swapped ? 0 : 1], Value_Immediate, Reg32::R8D);
                    auto dst = checkValue32(instr.dst[0], Value_Memory);

                    if(src.index() && dst.index())
                    {
                        // we lied about being able to use swapped sources
                        auto rmDst = std::get<RMOperand>(dst);
                        if(swapped)
                        {
                            // src1 == dst
                            if(std::holds_alternative<uint32_t>(src))
                            {
                                // immediate src0
                                // do negate for 0 - x?
                                auto tmp = mapReg32(0);
                                builder.mov(*tmp, rmDst);
                                builder.mov(rmDst, std::get<uint32_t>(src));
                                src = RMOperand{*tmp};
                            }
                            else if(instr.src[1] == 0)
                            {
                                // immediate src1, move src to dest
                                builder.mov(rmDst, std::get<RMOperand>(src).getReg32());
                                src = *getLastImmLoad(); 
                            }
                            else
                            {
                                // both reg sources
                                // save dst, move src to dst
                                auto tmp = mapReg32(0);
                                builder.mov(*tmp, rmDst);
                                builder.mov(rmDst, std::get<RMOperand>(src).getReg32());                                
                                src = RMOperand{*tmp};
                            }
                        }

                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            if(imm < 0x80)
                                builder.subD(rmDst, static_cast<int8_t>(imm));
                            else
                                builder.sub(rmDst, imm);
                        }
                        else
                            builder.sub(rmDst, std::get<RMOperand>(src).getReg32());

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R11B);

                        // flags
                        assert(!(instr.flags & GenOp_WriteFlags) || !rmDst.isMem());
                        setFlags32(rmDst.getReg32(), Reg32::R11D, instr.flags, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::SubtractWithCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkReg32(instr.src[1], Reg32::R8D);
                    auto dst = checkReg32(instr.dst[0], Reg32::R9D);

                    if(src && dst)
                    {
                        // carry in (and clear it)
                        builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                        builder.cmc(); // inverted

                        builder.sbb(*dst, *src);

                        // copy _inverted_ C flag if we need it
                        // assuming arm inverted carry flag
                        if(writesFlag(instr.flags, SourceFlagType::Carry) && writesFlag(instr.flags, SourceFlagType::Overflow))
                            builder.setcc(Condition::AE, Reg8::R11B);

                        // flags
                        setFlags32(*dst, Reg32::R11D, instr.flags, true);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Xor:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkValue32(instr.src[1], Value_Immediate);
                    auto dst = checkReg32(instr.dst[0]);

                    if(src.index() && dst)
                    {
                        if(std::holds_alternative<uint32_t>(src))
                        {
                            auto imm = std::get<uint32_t>(src);
                            // use sign extended if imm < 0x80? 
                            builder.xor_(*dst, imm);
                        }
                        else
                            builder.xor_(*dst, std::get<RMOperand>(src).getReg32());

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        setFlags32(*dst, {}, instr.flags);
                        
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Not:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    if(auto dst = checkReg32(instr.dst[0]))
                    {
                        builder.not_(*dst);

                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        assert(!writesFlag(instr.flags, SourceFlagType::Carry));
                        setFlags32(*dst, {}, instr.flags, false, 0); // doesn't affect flags
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();
                badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateLeftCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();
                badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRight:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::rorCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::ror), preOp, true))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = flagWriteMask(SourceFlagType::Carry); // only have carry
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::RotateRightCarry:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();
                badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftLeft:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shlCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shl), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightArith:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::sarCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::sar), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::ShiftRightLogic:
            {
                auto regSize = sourceInfo.registers[instr.src[0]].size;

                checkSingleSource();

                if(regSize == 32)
                {
                    auto src = checkRegOrImm8(instr.src[1]);
                    auto dst = checkReg32(instr.dst[0]);

                    auto preOp = [this, &instr](X86Builder &builder)
                    {
                        // to handle possibly preserved carry
                        if(writesFlag(instr.flags, SourceFlagType::Carry))
                            builder.btr(*mapReg32(flagsReg), getFlagInfo(SourceFlagType::Carry).bit);
                    };

                    if(doRegImmShift32(builder, dst, src, std::mem_fn<void(Reg32)>(&X86Builder::shrCL), std::mem_fn<void(Reg32, uint8_t)>(&X86Builder::shr), preOp))
                    {
                        assert(!writesFlag(instr.flags, SourceFlagType::Overflow));
                        auto setFlags = 0xF0;
                        if(std::holds_alternative<Reg8>(src))
                            setFlags = flagWriteMask(SourceFlagType::Carry); // if the src is a reg, it might be 0 (doesn't affect flags)
    
                        setFlags32(*dst, {}, instr.flags, false, setFlags);
                    }
                }
                else
                    badRegSize(regSize);
                break;
            }

            case GenOpcode::Jump:
            {
                auto condition = static_cast<GenCondition>(instr.src[0]);
                auto regSize = sourceInfo.registers[instr.src[1]].size;

                bool isExit = instr.flags & GenOp_Exit;

                if(regSize == 32)
                {
                    auto src = checkValue32(instr.src[1], Value_Immediate, Reg32::R8D);
                    if(src.index())
                    {
                        callRestoreIfNeeded(builder, needCallRestore);

                        assert(isExit || std::holds_alternative<uint32_t>(src)); // shouldn't be any non-exit jumps with unknown addr

                        uint8_t *branchPtr = nullptr;
                        bool flagSet = false;

                        // condition
                        if(condition != GenCondition::Always)
                        {
                            syncCyclesExecuted();

                            std::variant<Reg8, Reg32> f;

                            if(regSize == 32)
                                f = *mapReg32(flagsReg);// really should not get here without a valid flags reg

                            auto testFlag = [this, &builder, &f](SourceFlagType flag)
                            {
                                std::visit([this, &builder, flag](auto &&r){builder.test(r, 1 << getFlagInfo(flag).bit);}, f);
                            };

                            flagSet = !(static_cast<int>(condition) & 1); // even conds are set, odd are clear

                            switch(condition)
                            {
                                case GenCondition::Equal:
                                case GenCondition::NotEqual:
                                    testFlag(SourceFlagType::Zero);
                                    break;
                                case GenCondition::CarrySet:
                                case GenCondition::CarryClear:
                                    testFlag(SourceFlagType::Carry);
                                    break;
                                case GenCondition::Negative:
                                case GenCondition::Positive:
                                    testFlag(SourceFlagType::Negative);
                                    break;
                                case GenCondition::OverflowSet:
                                case GenCondition::OverflowClear:
                                    testFlag(SourceFlagType::Overflow);
                                    break;

                                case GenCondition::Higher:
                                case GenCondition::LowerSame:
                                {
                                    // C && !Z (higher)

                                    // (flags >> C.bit) & (Z.bit | C.bit) == 1
                                    assert(getFlagInfo(SourceFlagType::Zero).bit > getFlagInfo(SourceFlagType::Carry).bit);

                                    auto carryBit = getFlagInfo(SourceFlagType::Carry).bit;
                                    auto mask = (1 << (getFlagInfo(SourceFlagType::Zero).bit - carryBit)) | 1;

                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, carryBit);
                                    builder.and_(Reg8::R11B, mask);
                                    builder.cmp(Reg8::R11B, 1);

                                    flagSet = !flagSet; // oops, backwards
                                    break;
                                }

                                case GenCondition::GreaterEqual:
                                case GenCondition::LessThan:
                                {
                                    // N == V (greater equal)
                                    // N != V (less than)

                                    // shift N to where V is
                                    assert(getFlagInfo(SourceFlagType::Negative).bit > getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, getFlagInfo(SourceFlagType::Negative).bit - getFlagInfo(SourceFlagType::Overflow).bit);
                                    
                                    // xor
                                    builder.xor_(Reg32::R11D, std::get<Reg32>(f));
                                    
                                    flagSet = !flagSet; // oops, backwards
                                    f = Reg32::R11D;
                                    testFlag(SourceFlagType::Overflow); // test the result
                                    break;
                                }

                                case GenCondition::GreaterThan:
                                case GenCondition::LessThanEqual:
                                {
                                    // N == V && !Z (greater than)
                                    // N != V || Z (less than equal)

                                    // shift N to where V is
                                    assert(getFlagInfo(SourceFlagType::Negative).bit > getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.mov(Reg32::R11D, std::get<Reg32>(f));
                                    builder.shr(Reg32::R11D, getFlagInfo(SourceFlagType::Negative).bit - getFlagInfo(SourceFlagType::Overflow).bit);
                                    
                                    // xor
                                    builder.xor_(Reg32::R11D, std::get<Reg32>(f));

                                    // test result and Z
                                    auto mask = (1 << getFlagInfo(SourceFlagType::Zero).bit) | (1 << getFlagInfo(SourceFlagType::Overflow).bit);
                                    builder.test(Reg32::R11D, mask);

                                    flagSet = !flagSet; // oops, backwards
                                    break;
                                }

                                default:
                                    printf("unhandled cond %i\n", static_cast<int>(condition));
                                    err = true;
                            }

                            branchPtr = builder.getPtr();
                            builder.jcc(Condition::E, 1); // patch later
                        }

                        // need to sync cycles *before* the jump out
                        if(instrCycles)
                        {
                            while(instrCycles--)
                                cycleExecuted();
                        }
                        syncCyclesExecuted();

                        callRestoreIfNeeded(builder, needCallRestore); // we might have just done another cycleExecuted call

                        // set pc if we're exiting
                        if(isExit)
                        {
                            if(std::holds_alternative<uint32_t>(src))
                                builder.mov(pcReg32, std::get<uint32_t>(src) + sourceInfo.pcPrefetch);
                            else if(std::get<RMOperand>(src).getReg32() != pcReg32)
                            {
                                auto reg32 = std::get<RMOperand>(src).getReg32();
                                if(sourceInfo.pcPrefetch)
                                    builder.lea(pcReg32, {static_cast<Reg64>(reg32), sourceInfo.pcPrefetch});
                                else
                                    builder.mov(pcReg32, reg32);
                            }
                        }
                        else // or sub cycles early (we jump past the usual code that does this)
                            builder.sub(Reg32::EDI, static_cast<int8_t>(cyclesThisInstr));

                        // don't update twice for unconditional branches
                        if(condition == GenCondition::Always)
                            cyclesThisInstr = 0;

                        auto it = std::holds_alternative<uint32_t>(src) ? branchTargets.find(std::get<uint32_t>(src)) : branchTargets.end();

                        if(it != branchTargets.end())
                        {
                            // backwards branch
                            builder.jmp(it->second - builder.getPtr());
                        }
                        else
                        {
                            if(!isExit)
                                forwardBranchesToPatch.emplace(std::get<uint32_t>(src), builder.getPtr());

                            // forwards branch or exit
                            if(isExit && (instr.flags & GenOp_Call) && (instIt + 1) != endInstr)
                                builder.call(exitForCallPtr - builder.getPtr()); // call, save when exiting
                            else
                                builder.jmp(exitPtr - builder.getPtr(), !isExit); // patched later if not exit
                        }

                        // patch the condition jump
                        if(branchPtr && !builder.getError())
                        {
                            auto off = builder.getPtr() - branchPtr - 2;
                            builder.patch(branchPtr, branchPtr + 2);
                            builder.jcc(flagSet ? Condition::E : Condition::NE, off);
                            builder.endPatch();
                        }
                    }
                }
                else
                    badRegSize(regSize);

                break;
            }

            default:
                printf("unhandled gen op %i\n", static_cast<int>(instr.opcode));
                err = true;
        }

        if(builder.getError())
            break;

        // failed but builder still ok
        if(err)
        {
            builder.resetPtr(opStartPtr);
            builder.mov(pcReg32, pc - instr.len + sourceInfo.pcPrefetch);
            builder.jmp(exitPtr - builder.getPtr());
            break;
        }

        // sync cycle count from stack at the end of the op, or if something updates cycles before that
        if(stackCycleCount && (instrCycles || instr.len || (instIt + 1)->opcode == GenOpcode::Jump))
        {
            callRestoreIfNeeded(builder, Reg16::DI, needCallRestore);

            // add/sub the cycles from the stack
            builder.mov(Reg32::R8D, {Reg64::RSP, needCallRestore * 8});

            int offset = reinterpret_cast<uintptr_t>(sourceInfo.cycleCount) - reinterpret_cast<uintptr_t>(cpuPtr);
            builder.add({cpuPtrReg, offset}, Reg32::R8D);

            builder.sub(Reg32::EDI, Reg32::R8D);
            stackCycleCount = false;
        }

        // additional cycles
        if(instrCycles > 0)
        {
            while(instrCycles--)
                cycleExecuted();
        }

        // check if this is the end of the source instruction (pc incremented)
        newEmuOp = instr.len != 0;

        if(instIt + 1 == endInstr && (instr.opcode != GenOpcode::Jump || static_cast<GenCondition>(instr.src[0]) != GenCondition::Always))
        {
            // ending on a non-jump probably means this was an incomplete block
            // add an exit
            callRestoreIfNeeded(builder, needCallRestore);
            syncCyclesExecuted();
            builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
            builder.jmp(exitPtr - builder.getPtr());
        }

        // check cycle count if this is the last part of en emulated op
        // ... but not on the last op, that should always exit anyway
        // ... or exits unless followed by a branch target
        auto nextInstr = instIt + 1;
        bool isCond = instr.opcode == GenOpcode::Jump && static_cast<GenCondition>(instr.src[0]) != GenCondition::Always;
        bool shouldSkip = nextInstr == endInstr || ((instr.flags & GenOp_Exit) && !isCond && !(nextInstr->flags & GenOp_BranchTarget));
        if(newEmuOp && !shouldSkip)
        {
            // might exit, sync
            callRestoreIfNeeded(builder, needCallRestore);
            syncCyclesExecuted();

            // cycles -= executed
            if(cyclesThisInstr) // 0 means we already did the sub
                builder.sub(Reg32::EDI, static_cast<int8_t>(cyclesThisInstr));

            lastInstrCycleCheck = builder.getPtr(); // save in case the next instr is a branch target

            // if <= 0 exit
            builder.jcc(Condition::G, 10);
            builder.mov(pcReg32, pc + sourceInfo.pcPrefetch);
            builder.call(saveAndExitPtr - builder.getPtr());

            cyclesThisInstr = 0;
        }

        if(newEmuOp)
            numInstructions++;
    }

    if(builder.getError())
    {
        printf("recompile @%04X failed due to error (out of space?)\n", startPC);
        return false;
    }

    if(numInstructions == 0)
    {
#ifdef RECOMPILER_DEBUG
        printf("recompile @%04X failed to handle any instructions\n", startPC);
#endif
        return false;
    }

    auto endPtr = builder.getPtr();

#ifdef RECOMPILER_DEBUG
    int len = endPtr - codePtr;

    //debug
    printf("recompile @%04X generated %i bytes (%i instructions)\ncode:", startPC, len, numInstructions);

    for(auto p = codePtr; p != endPtr; p++)
        printf(" %02X", *p);

    printf("\n");
    printf("(addr %p->%p)\n", codePtr, endPtr);
#endif

    codePtr = endPtr;

    return true;
}

uint8_t *X86Target::compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize)
{
    X86Builder builder(codeBuf, codeBuf + codeBufSize);

    // prologue
    builder.push(Reg64::RBP);

    // save
    builder.push(Reg64::R12);
    builder.push(Reg64::R13);
    builder.push(Reg64::R14);
    if(saveR15)
        builder.push(Reg64::R15);

    builder.push(Reg64::RBX);

#ifdef _WIN32
    builder.push(Reg64::RSI);
    builder.push(Reg64::RDI);

    builder.mov(Reg64::RDI, argumentRegs64[0]); // move cycle count
#endif

    if(sourceInfo.readMem8)
        builder.sub(Reg64::RSP, 16); // save some space if we're using the more advanced memory access

    builder.mov(Reg64::R10, argumentRegs64[1]); // code ptr

    // store pointer to CPU
    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpuPtr);
    builder.mov(cpuPtrReg, cpuPtrInt);


    // load emu regs
    uint8_t i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 32);

            if(auto reg32 = mapReg32(i))
            {
                if(reg.size == 32)
                    builder.mov(*reg32, {cpuPtrReg, reg.cpuOffset});
            }
        }

        i++;
    }

    // jump to code
    builder.jmp(Reg64::R10);

    // exit setting the call flag ... and saving ip
    exitForCallPtr = builder.getPtr();
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.exitCallFlag));
    builder.mov({Reg64::R11, 0}, uint8_t(1));

    // exit saving ip
    saveAndExitPtr = builder.getPtr();
    builder.pop(Reg64::R10); // ret address (this is called)
    builder.mov(Reg64::R11, reinterpret_cast<uintptr_t>(sourceInfo.savedExitPtr));
    builder.mov({Reg64::R11, 0}, Reg64::R10);

    // just exit
    exitPtr = builder.getPtr();

    // save emu regs
    i = 0;
    for(auto &reg : sourceInfo.registers)
    {
        if(reg.cpuOffset != 0xFFFF)
        {
            assert(reg.size == 16 || reg.size == 32);

            if(reg.size == 32)
            {
                if(auto reg32 = mapReg32(i))
                    builder.mov({cpuPtrReg, reg.cpuOffset}, *reg32);
            }
        }

        i++;
    }

    // save emu pc
    if(sourceInfo.pcSize == 16)
        builder.mov({cpuPtrReg, sourceInfo.pcOffset}, pcReg16);
    else if(sourceInfo.pcSize == 32)
        builder.mov({cpuPtrReg, sourceInfo.pcOffset}, pcReg32);

    // restore

    if(sourceInfo.readMem8)
        builder.add(Reg64::RSP, 16);

#ifdef _WIN32
    builder.pop(Reg64::RDI);
    builder.pop(Reg64::RSI);
#endif

    builder.pop(Reg64::RBX);

    if(saveR15)
        builder.pop(Reg64::R15);
    builder.pop(Reg64::R14);
    builder.pop(Reg64::R13);
    builder.pop(Reg64::R12);

    // epilogue
    builder.pop(Reg64::RBP);
    builder.ret();

    // generate trampolines for calls
    auto bounce = [&builder](auto *func)
    {
        uint8_t *ret = nullptr;
        if(func)
        {
            ret = builder.getPtr();
            builder.mov(Reg64::RAX, reinterpret_cast<uintptr_t>(func));
            builder.jmp(Reg64::RAX);
        }
        return ret;
    };

    readMem8Ptr = bounce(sourceInfo.readMem8);
    readMem16Ptr = bounce(sourceInfo.readMem16);
    readMem32Ptr = bounce(sourceInfo.readMem32);
    writeMem8Ptr = bounce(sourceInfo.writeMem8);
    writeMem16Ptr = bounce(sourceInfo.writeMem16);
    writeMem32Ptr = bounce(sourceInfo.writeMem32);

#ifdef RECOMPILER_DEBUG
    int len = builder.getPtr() - codeBuf;

    //debug
    printf("generated %i bytes for entry/exit\ncode:", len);

    for(auto p = codeBuf; p != codeBuf + len; p++)
        printf(" %02X", *p);

    printf("\n");
#endif

    auto ret = codeBuf;

    codeBuf = builder.getPtr();

    return ret;
}

std::optional<Reg8> X86Target::mapReg8(uint8_t index)
{
    // remap aliases
    auto &reg = sourceInfo.registers[index];
    if(reg.alias)
        index = reg.alias;

    if(auto reg32 = mapReg32(index))
    {
        // default to low
        if(!reg.alias || reg.aliasMask == 0xFF)
        {
            // SPL/BPL/SIL/DIL (require REX)
            if(*reg32 == Reg32::ESP || *reg32 == Reg32::EBP || *reg32 == Reg32::ESI || *reg32 == Reg32::EDI)
                return static_cast<Reg8>(static_cast<int>(*reg32) + 0x10);
    
            return static_cast<Reg8>(*reg32);
        }

        // handle high aliases
        if(reg.aliasMask == 0xFF00 && (*reg32 == Reg32::EAX || *reg32 == Reg32::ECX || *reg32 == Reg32::EDX || *reg32 == Reg32::EBX))
            return static_cast<Reg8>(static_cast<int>(*reg32) + 4);

        return {};
    }

    return {};
}

std::optional<Reg16> X86Target::mapReg16(uint8_t index)
{
    if(auto reg32 = mapReg32(index))
        return static_cast<Reg16>(*reg32);

    return {};
}

std::optional<Reg32> X86Target::mapReg32(uint8_t index)
{
    if(sourceInfo.registers[index].alias)
        return {};

    auto alloc = regAlloc.find(index);
    if(alloc != regAlloc.end())
        return alloc->second;

    return {};
}

std::optional<Reg64> X86Target::mapReg64(uint8_t index)
{
    if(auto reg32 = mapReg32(index))
        return static_cast<Reg64>(*reg32);

    return {};
}

SourceFlagInfo &X86Target::getFlagInfo(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    assert(flagMap[iFlag] != 0xFF);

    return sourceInfo.flags[flagMap[iFlag]];
}

uint8_t X86Target::flagWriteMask(SourceFlagType flag)
{
    auto iFlag = static_cast<int>(flag);
    if(flagMap[iFlag] == 0xFF) // source does not have this flag
        return 0;

    return 1 << (flagMap[iFlag] + 4);
}

bool X86Target::writesFlag(uint16_t opFlags, SourceFlagType flag)
{
    return opFlags & flagWriteMask(flag);
}


bool X86Target::needStackAlign() const
{
    // if R15 is saved the stack is misaligned before saving anything
    // otherwise if we're saving an odd number of registiers it's misaligned
    return !!(numSavedRegs % 2) != saveR15;
}

bool X86Target::isCallSaved(Reg16 reg) const
{
    return callSaveIndex(reg) != -1;
}

bool X86Target::isCallSaved(Reg8 reg) const
{
    return callSaveIndex(reg) != -1;
}

int X86Target::callSaveIndex(Reg16 reg) const
{
    for(size_t i = 0; i < numSavedRegs; i++)
    {
        if(reg == static_cast<Reg16>(callSavedRegs[i]))
            return static_cast<int>(i);
    }
    
    return -1;
}

int X86Target::callSaveIndex(Reg8 reg) const
{
    Reg16 mappedReg = static_cast<Reg16>(reg);
    auto iReg = static_cast<int>(reg);

    if(iReg >= 16) // SPL/BPL/SIL/DIL
        mappedReg = static_cast<Reg16>(iReg - 16);
    else if(isXHReg(reg))
        mappedReg = static_cast<Reg16>(iReg - 4);

    return callSaveIndex(mappedReg);
}

void X86Target::callSaveIfNeeded(X86Builder &builder, int &saveState) const
{
    int numRegs = static_cast<int>(numSavedRegs);

    if(saveState == numRegs)
        return;

    for(int i = saveState; i < numRegs; i++)
        builder.push(callSavedRegs[i]);

    if(needStackAlign())
        builder.sub(Reg64::RSP, 8); // align stack

#ifdef _WIN32
    builder.sub(Reg64::RSP, 32); // shadow space
#endif

    saveState = numRegs;
}

void X86Target::callRestore(X86Builder &builder, int &saveState, int toIndex) const
{
    if(saveState <= toIndex)
        return;

    int numRegs = static_cast<int>(numSavedRegs);

    assert(toIndex < numRegs);

#ifdef _WIN32
    if(saveState == numRegs)
        builder.add(Reg64::RSP, 32); // shadow space
#endif

    if(saveState == numRegs && needStackAlign())
        builder.add(Reg64::RSP, 8); // alignment

    for(int i = saveState - 1; i >= toIndex; i--)
        builder.pop(callSavedRegs[i]);

    saveState = toIndex;
}

void X86Target::callRestore(X86Builder &builder, Reg8 dstReg, bool zeroExtend, bool signExtend) const
{
    assert(dstReg != Reg8::DIL); // no

    assert(isCallSaved(dstReg));

    assert(!zeroExtend || !signExtend); // both makes no sense

#ifdef _WIN32
    builder.add(Reg64::RSP, 32); // shadow space
#endif

    if(needStackAlign())
        builder.add(Reg64::RSP, 8); // alignment

    // pop everything except RAX
    for(unsigned i = numSavedRegs - 1; i > 0; i--)
        builder.pop(callSavedRegs[i]);

    assert(callSavedRegs[0] == Reg64::RAX);

    // mov ret val (if not going to RAX)
    if(dstReg != Reg8::AL && dstReg != Reg8::AH)
    {
        if(signExtend)
            builder.movsx(static_cast<Reg32>(dstReg), Reg8::AL);
        else if(zeroExtend)
            builder.movzx(static_cast<Reg32>(dstReg), Reg8::AL);
        else
            builder.mov(dstReg, Reg8::AL);
        builder.pop(Reg64::RAX);
    }
    else if(dstReg == Reg8::AH) // TODO: having a worse case for A is not great
    {
        assert(!zeroExtend && !signExtend);
        builder.mov(Reg8::AH, Reg8::AL);
        builder.pop(Reg64::R10);
        builder.mov(Reg8::AL, Reg8::R10B); // restore low byte
    }
    else if(dstReg == Reg8::AL)// ... though this is the worst case... (AL == F, so unlikely)
    {
        builder.pop(Reg64::R10);
        
        if(signExtend)
            builder.movsx(Reg32::EAX, Reg8::AL);
        else if(!zeroExtend)
        {
            // EAX = EAX + (R10D & 0xFF00)
            builder.and_(Reg32::R10D, 0xFF00u);
            builder.movzx(Reg32::EAX, Reg8::AL);
            builder.add(Reg32::EAX, Reg32::R10D); // TODO: OR? (haven't added that to builder yet)
        }
    }
}

void X86Target::callRestoreIfNeeded(X86Builder &builder, int &saveState) const
{
    callRestore(builder, saveState, 0);
}

// restore if it would affect this register
// takes a variant so it can be passed the result of checkRegOrImm
void X86Target::callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg8, uint8_t> val, int &saveState) const
{
    if(!std::holds_alternative<Reg8>(val))
        return; // not a register

    auto index = callSaveIndex(std::get<Reg8>(val));

    if(index != -1)
        callRestore(builder, saveState, index);
}

void X86Target::callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg16, uint16_t> val, int &saveState) const
{
    if(!std::holds_alternative<Reg16>(val))
        return;

    auto index = callSaveIndex(std::get<Reg16>(val));

    if(index != -1)
        callRestore(builder, saveState, index);
}
