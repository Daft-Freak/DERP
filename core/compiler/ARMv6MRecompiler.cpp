#include <cassert>
#include <cstdio>
#include <utility>

#ifdef __linux__
#include <sys/mman.h>
#include <unistd.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "ARMv6MRecompiler.h"

#include "ARMv6MCore.h"
#include "MemoryBus.h"

// reg 0 is special tmp register
enum class GenReg
{
    Temp = 0, // special temp

    R0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    // R15 = PC

    CPSR,

    Control,
    PriMask,

    Temp2, // used by POP, LDM
};

uint16_t getRegOffset(void *cpuPtr, uint8_t reg)
{
    auto cpu = reinterpret_cast<ARMv6MCore *>(cpuPtr);

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(cpu);

    auto genReg = static_cast<GenReg>(reg);

    if(reg < 16)
    {
        uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu->regs) - cpuPtrInt;

        auto mapped = cpu->mapReg(static_cast<ARMv6MCore::Reg>(reg - 1));

        return regsOffset + static_cast<int>(mapped) * 4;
    }

    if(genReg == GenReg::Control)
        return reinterpret_cast<uintptr_t>(&cpu->control) - cpuPtrInt;
    if(genReg == GenReg::PriMask)
        return reinterpret_cast<uintptr_t>(&cpu->primask) - cpuPtrInt;

    assert(!"invalid register!");
    return 0;
}

ARMv6MRecompiler::ARMv6MRecompiler(ARMv6MCore &cpu) : cpu(cpu)
{
    SourceInfo sourceInfo{};

    auto cpuPtrInt = reinterpret_cast<uintptr_t>(&cpu);

    uint16_t regsOffset = reinterpret_cast<uintptr_t>(&cpu.regs) - cpuPtrInt;

    sourceInfo.registers.emplace_back(SourceRegInfo{"tmp", 32, SourceRegType::Temp, 0, 0, 0xFFFF});

    // main regs
    sourceInfo.registers.emplace_back(SourceRegInfo{"R0 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R1 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R2 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R3 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R4 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R5 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R6 ", 32, SourceRegType::General, 0, 0, regsOffset});
    regsOffset += 4;
    sourceInfo.registers.emplace_back(SourceRegInfo{"R7 ", 32, SourceRegType::General, 0, 0, regsOffset});
    
    // >= R8 may require mapping
    sourceInfo.registers.emplace_back(SourceRegInfo{"R8 ", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R9 ", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R10", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R11", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R12", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"R13", 32, SourceRegType::General, 0, 0, 0xFFFF}); // SP
    sourceInfo.registers.emplace_back(SourceRegInfo{"R14", 32, SourceRegType::General, 0, 0, 0xFFFF}); // LR
    // R15 is PC

    regsOffset = reinterpret_cast<uintptr_t>(&cpu.cpsr) - cpuPtrInt;
    sourceInfo.registers.emplace_back(SourceRegInfo{"PSR", 32, SourceRegType::Flags, 0, 0, regsOffset});

    // CONTROL/PRIMASK (3 chars is not helping here)
    sourceInfo.registers.emplace_back(SourceRegInfo{"CTL", 32, SourceRegType::General, 0, 0, 0xFFFF});
    sourceInfo.registers.emplace_back(SourceRegInfo{"PMK", 32, SourceRegType::General, 0, 0, 0xFFFF});

    sourceInfo.registers.emplace_back(SourceRegInfo{"tm2", 32, SourceRegType::Temp, 0, 0, 0xFFFF});

    // condition flags
    sourceInfo.flags.emplace_back(SourceFlagInfo{'V', 28, SourceFlagType::Overflow});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'C', 29, SourceFlagType::Carry});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'Z', 30, SourceFlagType::Zero});
    sourceInfo.flags.emplace_back(SourceFlagInfo{'N', 31, SourceFlagType::Negative});

    sourceInfo.pcSize = 32;
    sourceInfo.pcPrefetch = 2;
    sourceInfo.pcOffset = reinterpret_cast<uintptr_t>(&cpu.regs[15]) - cpuPtrInt;

    sourceInfo.cycleMul = 1;

    sourceInfo.getRegOffset = getRegOffset;

    sourceInfo.exitCallFlag = &exitCallFlag;
    sourceInfo.savedExitPtr = &tmpSavedPtr;
    sourceInfo.cycleCount = &cycleCount;

    sourceInfo.readMem8 = reinterpret_cast<uint8_t (*)(void *, uint32_t, int &, uint8_t)>(ARMv6MRecompiler::readMem8);
    sourceInfo.readMem16 = reinterpret_cast<uint32_t (*)(void *, uint32_t, int &, uint8_t)>(ARMv6MRecompiler::readMem16);
    sourceInfo.readMem32 = reinterpret_cast<uint32_t (*)(void *, uint32_t, int &, uint8_t)>(ARMv6MRecompiler::readMem32);

    sourceInfo.writeMem8 = reinterpret_cast<int (*)(void *, uint32_t, uint8_t, int &, uint8_t, int)>(ARMv6MRecompiler::writeMem8);
    sourceInfo.writeMem16 = reinterpret_cast<int (*)(void *, uint32_t, uint16_t, int &, uint8_t, int)>(ARMv6MRecompiler::writeMem16);
    sourceInfo.writeMem32 = reinterpret_cast<int (*)(void *, uint32_t, uint32_t, int &, uint8_t, int)>(ARMv6MRecompiler::writeMem32);

    target.init(sourceInfo, &cpu);

#if defined(__linux__)
    // allocate some memory
    auto pageSize = sysconf(_SC_PAGE_SIZE);
    int numPages = 256 * 8;

    // FIXME: alloc RW, switch to RX
    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(mmap(0, codeBufSize, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));

    if(codeBuf == MAP_FAILED)
        perror("failed to allocate code buffer (mmap failed)");
#elif defined(_WIN32)
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    auto pageSize = sysInfo.dwPageSize;
    int numPages = 256;

    codeBufSize = pageSize * numPages;
    codeBuf = reinterpret_cast<uint8_t *>(VirtualAlloc(nullptr, codeBufSize, MEM_COMMIT, PAGE_EXECUTE_READWRITE));
#endif

    curCodePtr = codeBuf;

    for(auto &saved : savedExits)
        saved = {nullptr, 0, 0};
}

int ARMv6MRecompiler::run(int cyclesToRun)
{
    if(!codeBuf)
        return 0;

    int cyclesExecuted = 0;
    bool didIntr = false;

    while(true)
    {
        // calculate cycles to run
        auto cyclesToIntr = cpu.clock.getCyclesToTime(cpu.mem.getNextInterruptTime());

        int cycles = std::min(cyclesToRun - cyclesExecuted, static_cast<int>(cyclesToIntr));

        if(cycles <= 0)
            break;

        auto inPC = cpu.loReg(ARMv6MCore::Reg::PC);

        if(!attemptToRun(cycles, cyclesExecuted))
            break;

        cpu.clock.addCycles(cycleCount);

        // handle long branches
        auto outPC = cpu.loReg(ARMv6MCore::Reg::PC);
        if(inPC >> 24 != outPC >> 24)
            cpu.pcPtr = nullptr; // force remap later

        if(cpu.loReg(ARMv6MCore::Reg::PC) >> 28 == 0xF)
            cpu.handleExceptionReturn(cpu.loReg(ARMv6MCore::Reg::PC), true);

        // might have tried to change mode
        if(!(cpu.cpsr & ARMv6MCore::Flag_T))
        {
            if(cpu.loReg(ARMv6MCore::Reg::PC) & 1)
            {
                // nope, it's still thumb (we just did a bx)
                cpu.cpsr |= ARMv6MCore::Flag_T;
                cpu.loReg(ARMv6MCore::Reg::PC) &= ~1;
            }
            else
                abort(); // there is no ARM mode
        }

        didIntr = false;

        // CPU not running, stop
        if(cpu.sleeping)
            break;

        if(!(cpu.primask & 1) && cpu.needException)
        {
            int intrCycles = cpu.handleException();
            cyclesExecuted += intrCycles;
            cpu.clock.addCycles(intrCycles);

            if(intrCycles)
                didIntr = true;
        }

        // CPU update stuff
        auto curTime = cpu.clock.getTime();
        if(cpu.mem.getNextInterruptTime() <= curTime)
        {
            cpu.mem.peripheralUpdate(curTime, cpu.sleeping ? ~0u : cpu.nvicEnabled, &cpu);
            cpu.mem.calcNextInterruptTime();
        }
    }

    // if we executed anything, we need to update PC
    if(cyclesExecuted && !didIntr)
    {
        auto pc = cpu.loReg(ARMv6MCore::Reg::PC) - 2;

        if(!cpu.pcPtr)
            cpu.updateTHUMBPC(pc, true);
        else
        {
            auto thumbPCPtr = reinterpret_cast<const uint16_t *>(cpu.pcPtr + pc);
            cpu.decodeOp = *thumbPCPtr++;
            cpu.fetchOp = *thumbPCPtr;
        }
    }

    return cyclesExecuted;
}

bool ARMv6MRecompiler::attemptToRun(int cyclesToRun, int &cyclesExecuted)
{
    uint8_t *codePtr = nullptr;

    auto cpuPC = cpu.loReg(ARMv6MCore::Reg::PC) - 2;
    auto blockStartPC = cpuPC;

    // attempt to re-enter previous code
    int savedIndex = curSavedExit - 1;
    for(int i = 0; i < savedExitsSize; i++, savedIndex--)
    {
        // wrap
        if(savedIndex < 0)
            savedIndex += savedExitsSize;

        uint8_t *ptr;
        uint32_t pc;
        uint32_t startPC;
        std::tie(ptr, pc, startPC) = savedExits[savedIndex];

        if(pc == cpuPC && ptr)
        {
            codePtr = ptr;
            curSavedExit = savedIndex;

            cpu.loReg(ARMv6MCore::Reg::PC) = startPC + 2; // compiled code depends on PC pointing to the start of the block
            blockStartPC = startPC;

            savedExits[savedIndex] = {nullptr, 0, 0};
            break;
        }
    }

    if(!codePtr)
    {
        // lookup compiled code
        auto it = compiled.find(cpuPC);

        if(it == compiled.end())
        {
            if(!entryFunc)
                compileEntry();

            // attempt compile
            auto ptr = curCodePtr;
            auto startPtr = ptr;
            auto pc = cpuPC;

            GenBlockInfo genBlock;
            genBlock.flags = 0;

            convertTHUMBToGeneric(pc, genBlock);

            analyseGenBlock(cpuPC, pc, genBlock, target.getSourceInfo());

#ifdef RECOMPILER_DEBUG
            printf("analysed %04X-%04X (%zi instructions)\n", cpuPC, pc, genBlock.instructions.size());
            printGenBlock(cpuPC, genBlock, target.getSourceInfo());
            printf("\n\n");
#endif

            FuncInfo info{};

            info.cpsrMode = cpu.cpsr & 0x1F;

            if(target.compile(ptr, codeBuf + codeBufSize, cpuPC, genBlock))
            {
                info.startPtr = startPtr;
                info.endPtr = curCodePtr = ptr;
                info.endPC = pc;
            }
            
            it = compiled.emplace(cpuPC, info).first;

            // track range of code in RAM
            if(cpuPC >= 0x20000000)
            {
                bool inScratch = cpuPC >= 0x20040000;
                int index = inScratch ? 1 : 0;

                if(cpuPC < minRAMCode[index])
                    minRAMCode[index] = cpuPC;
                if(pc > maxRAMCode[index])
                    maxRAMCode[index] = pc;
            }

            // cache these iterators for invalidation checks
            ramStartIt[0] = compiled.lower_bound(0x20000000);
            ramStartIt[1] = compiled.lower_bound(0x20040000);
        }

        // reject code if compiled for a different CPU mode
        if(it->second.cpsrMode == (cpu.cpsr & 0x1F))
            codePtr = it->second.startPtr;
    }

    cycleCount = extraCycleCount = 0;

    // run the code if valid, or stop
    if(codePtr)
        entryFunc(cyclesToRun, codePtr);
    else
        return false;

    cyclesExecuted += cycleCount + extraCycleCount;

    // code exited with a saved address for re-entry, store PC for later
    if(tmpSavedPtr)
    {
        auto savedPC = cpu.loReg(ARMv6MCore::Reg::PC) - 2;

        // get return address
        if(exitCallFlag)
            savedPC = cpu.reg(ARMv6MCore::Reg::LR) & ~1;

        savedExits[curSavedExit++] = {tmpSavedPtr, savedPC, blockStartPC};
        curSavedExit %= savedExitsSize;

        tmpSavedPtr = nullptr;
        exitCallFlag = false;
    }

    return true;
}

void ARMv6MRecompiler::convertTHUMBToGeneric(uint32_t &pc, GenBlockInfo &genBlock)
{
    bool done = false;

    auto &mem = cpu.getMem();

    auto startPC = pc;
    auto maxBranch = pc;

    auto lowReg = [](int reg)
    {
        assert(reg < 8);

        return static_cast<GenReg>(reg + 1);
    };

    // TODO these are basically the same
    auto reg = [](int reg)
    {
        assert(reg < 15);

        return static_cast<GenReg>(reg + 1);
    };

    auto updateEnd = [&startPC, &maxBranch](uint32_t target)
    {
        maxBranch = std::max(maxBranch, target);
    };

    auto addInstruction = [&genBlock](GenOpInfo op, uint8_t len = 0, uint16_t flags = 0)
    {
        if(flags & GenOp_Exit)
            assert(op.opcode == GenOpcode::Jump);

        // move branch target flag up to the start of the original instruction
        if((flags & GenOp_BranchTarget) && !genBlock.instructions.empty() && !genBlock.instructions.back().len)
        {
            flags &= ~GenOp_BranchTarget;
            auto rit = genBlock.instructions.rbegin();
            for(;rit != genBlock.instructions.rend(); ++rit)
            {
                if(rit->len) // end of previous instr
                {
                    rit--; // the one after
                    break;
                }
            }

            // must be the first instruction
            if(rit == genBlock.instructions.rend())
                rit--;

            rit->flags |= GenOp_BranchTarget;
        }

        op.len = len;
        op.flags = flags;
        genBlock.instructions.emplace_back(std::move(op));
    };

    // helpers
    auto loadImm = [](uint32_t imm, int cycles = 0)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::LoadImm;
        ret.cycles = cycles;
        ret.imm = imm;

        return ret;
    };

    auto move = [](GenReg src, GenReg dst, int cycles = 0)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Move;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto load = [](int size, GenReg addr, GenReg dst, int cycles)
    {
        GenOpInfo ret{};
        if(size == 1)
            ret.opcode = GenOpcode::Load;
        else if(size == 2)
            ret.opcode = GenOpcode::Load2;
        else if(size == 4)
            ret.opcode = GenOpcode::Load4;

        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(addr);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto store = [](int size, GenReg addr, GenReg data, int cycles)
    {
        GenOpInfo ret{};
        if(size == 1)
            ret.opcode = GenOpcode::Store;
        else if(size == 2)
            ret.opcode = GenOpcode::Store2;
        else if(size == 4)
            ret.opcode = GenOpcode::Store4;

        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(addr);
        ret.src[1] = static_cast<uint8_t>(data);

        return ret;
    };

    auto alu = [](GenOpcode op, GenReg src0, GenReg src1, GenReg dst, int cycles = 0)
    {
        GenOpInfo ret{};
        ret.opcode = op;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src0);
        ret.src[1] = static_cast<uint8_t>(src1);
        ret.dst[0] = static_cast<uint8_t>(dst);

        return ret;
    };

    auto compare = [](GenReg src0, GenReg src1, int cycles)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Compare;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(src0);
        ret.src[1] = static_cast<uint8_t>(src1);

        return ret;
    };

    auto jump = [](GenCondition cond = GenCondition::Always, GenReg src = GenReg::Temp, int cycles)
    {
        GenOpInfo ret{};
        ret.opcode = GenOpcode::Jump;
        ret.cycles = cycles;
        ret.src[0] = static_cast<uint8_t>(cond);
        ret.src[1] = static_cast<uint8_t>(src);

        return ret;
    };

    // common patterns
    auto loadWithOffset = [&](int size, GenReg base, int offset, GenReg dst, int cycles, int len = 0, int flags = 0)
    {
        if(offset)
        {
            addInstruction(loadImm(offset * size));
            addInstruction(alu(GenOpcode::Add, base, GenReg::Temp, GenReg::Temp));
        }

        addInstruction(load(size, offset ? GenReg::Temp : base, dst, cycles), len, flags);
    };

    auto storeWithOffset = [&](int size, GenReg base, int offset, GenReg dst, int cycles, int len = 0, int flags = 0)
    {
        if(offset)
        {
            addInstruction(loadImm(offset * size));
            addInstruction(alu(GenOpcode::Add, base, GenReg::Temp, GenReg::Temp));
        }

        addInstruction(store(size, offset ? GenReg::Temp : base, dst, cycles), len, flags);
    };

    const int preserveV = ARMv6MCore::Flag_V >> 28;
    const int preserveC = ARMv6MCore::Flag_C >> 28;

    const int writeV = ARMv6MCore::Flag_V >> 24;
    const int writeC = ARMv6MCore::Flag_C >> 24;
    const int writeZ = ARMv6MCore::Flag_Z >> 24;
    const int writeN = uint32_t(ARMv6MCore::Flag_N) >> 24;

    auto pcPtr = reinterpret_cast<const uint16_t *>(std::as_const(mem).mapAddress(pc));

    // we don't handle prefetch here, hmm
    auto pcSCycles = cpu.pcSCycles, pcNCycles = cpu.pcNCycles;

    while(!done)
    {
        uint16_t opcode = *pcPtr++;
        pc += 2;

        switch(opcode >> 12)
        {
            case 0x0: // format 1, move shifted
            case 0x1: // formats 1-2
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto srcReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                if(instOp == 3) // format 2, add/sub
                {
                    bool isImm = opcode & (1 << 10);
                    bool isSub = opcode & (1 << 9);

                    auto src1Reg = GenReg::Temp;

                    if(isImm)
                        addInstruction(loadImm((opcode >> 6) & 7));
                    else
                        src1Reg = lowReg((opcode >> 6) & 7);

                    if(isSub)
                        addInstruction(alu(GenOpcode::Subtract, srcReg, src1Reg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                    else
                        addInstruction(alu(GenOpcode::Add, srcReg, src1Reg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                }
                else // format 1, move shifted register
                {
                    auto offset = (opcode >> 6) & 0x1F;

                    switch(instOp)
                    {
                        case 0: // LSL
                        {
                            if(offset == 0)
                                addInstruction(move(srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            else
                            {
                                addInstruction(loadImm(offset));
                                addInstruction(alu(GenOpcode::ShiftLeft, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            }
                            break;
                        }

                        case 1: // LSR
                            addInstruction(loadImm(offset ? offset : 32));
                            addInstruction(alu(GenOpcode::ShiftRightLogic, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            break;

                        case 2: // ASR
                            addInstruction(loadImm(offset ? offset : 32));
                            addInstruction(alu(GenOpcode::ShiftRightArith, srcReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | writeC | writeZ | writeN);
                            break;
                    }
                }
                
                break;
            }

            case 0x2: // format 3, mov/cmp immediate
            case 0x3: // format 3, add/sub immediate
            {
                auto instOp = (opcode >> 11) & 0x3;
                auto dstReg = lowReg((opcode >> 8) & 7);

                addInstruction(loadImm(opcode & 0xFF));

                switch(instOp)
                {
                    case 0: // MOV
                        addInstruction(move(GenReg::Temp, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                        break;
                    case 1: // CMP
                        addInstruction(compare(dstReg, GenReg::Temp, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 2: // ADD
                        addInstruction(alu(GenOpcode::Add, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                    case 3: // SUB
                        addInstruction(alu(GenOpcode::Subtract, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                        break;
                }

                break;
            }

            case 0x4: // formats 4-6
            {
                if(opcode & (1 << 11)) // format 6, PC-relative load
                {
                    // this is almost certainly going to cause more timing problems
                    auto dstReg = lowReg((opcode >> 8) & 7);
                    uint8_t word = opcode & 0xFF;
                    auto addr = ((pc + 2) & ~2) + (word << 2);

                    int cycles = pcSCycles + 1;

                    addInstruction(loadImm(mem.read<uint32_t>(&cpu, addr, cycles)));
                    addInstruction(move(GenReg::Temp, dstReg, cycles), 2);
                }
                else if(opcode & (1 << 10)) // format 5, Hi reg/branch exchange
                {
                    auto op = (opcode >> 8) & 3;
                    bool h1 = opcode & (1 << 7);
                    bool h2 = opcode & (1 << 6);

                    auto srcReg = ((opcode >> 3) & 7) + (h2 ? 8 : 0);
                    auto dstReg = (opcode & 7) + (h1 ? 8 : 0);

                    switch(op)
                    {
                        case 0: // ADD
                            if(srcReg == 15) // read pc
                            {
                                assert(dstReg != 15);
                                addInstruction(loadImm(pc + 2));
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), GenReg::Temp, reg(dstReg), pcSCycles), 2);
                            }
                            else if(dstReg == 15)
                            {
                                printf("unhandled add pc in convertToGeneric %i -> %i\n", srcReg, dstReg);
                                done = true;
                            }
                            else if(dstReg == 13) // dest SP
                            {
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), reg(srcReg), reg(dstReg), pcSCycles));
                                // mask out low bits
                                addInstruction(loadImm(~3u));
                                addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)), 2);
                            }
                            else
                                addInstruction(alu(GenOpcode::Add, reg(dstReg), reg(srcReg), reg(dstReg), pcSCycles), 2);
                            break;
                        case 1: // CMP
                            if(srcReg == 15 || dstReg == 15)
                            {
                                printf("unhandled cmp pc in convertToGeneric %i -> %i\n", srcReg, dstReg);
                                done = true;
                            }
                            else
                                addInstruction(compare(reg(dstReg), reg(srcReg), pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;

                        case 2: // MOV
                        {
                            if(srcReg == 15) // read pc
                            {
                                assert(dstReg != 15);
                                addInstruction(loadImm(pc + 2));
                                addInstruction(move(GenReg::Temp, reg(dstReg), pcSCycles), 2);
                            }
                            else if(dstReg == 15)
                            {
                                
                                // clear low bit (no interworking here)
                                addInstruction(loadImm(~1u));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp, reg(srcReg), GenReg::Temp));
                                addInstruction(jump(GenCondition::Always, GenReg::Temp, pcNCycles + pcSCycles * 2), 2);
                                if(pc > maxBranch)
                                    done = true;
                            }
                            else if(dstReg == 13) // dest SP
                            {
                                addInstruction(move(reg(srcReg), reg(dstReg), pcSCycles));
                                // mask out low bits
                                addInstruction(loadImm(~3u));
                                addInstruction(alu(GenOpcode::And, reg(dstReg), GenReg::Temp, reg(dstReg)), 2);
                            }
                            else
                                addInstruction(move(reg(srcReg), reg(dstReg), pcSCycles), 2);
                            break;
                        }

                        case 3: // BX
                        {
                            if(srcReg == 15)
                            {
                                // there is no reason to do this, it would fault
                                printf("unhandled BX PC in convertToGeneric\n");
                                done = true;
                            }
                            else
                            {
                                if(h1) // BLX
                                {
                                    addInstruction(loadImm(pc | 1));
                                    addInstruction(move(GenReg::Temp, GenReg::R14));
                                }

                                // clear T flag
                                addInstruction(loadImm(~ARMv6MCore::Flag_T));
                                addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));

                                // jump (without clearing low bit, will use to correct flags later)
                                addInstruction(jump(GenCondition::Always, reg(srcReg), pcNCycles + pcSCycles * 2), 2);

                                if(pc > maxBranch)
                                    done = true;
                            }
                            break;
                        }
                    }
                }
                else // format 4, alu
                {
                    auto instOp = (opcode >> 6) & 0xF;
                    auto srcReg = lowReg((opcode >> 3) & 7);
                    auto dstReg = lowReg(opcode & 7);

                    switch(instOp)
                    {
                        case 0x0: // AND
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x1: // EOR
                            addInstruction(alu(GenOpcode::Xor, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x2: // LSL
                            addInstruction(alu(GenOpcode::ShiftLeft, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x3: // LSR
                            addInstruction(alu(GenOpcode::ShiftRightLogic, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x4: // ASR
                            addInstruction(alu(GenOpcode::ShiftRightArith, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x5: // ADC
                            addInstruction(alu(GenOpcode::AddWithCarry, dstReg, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0x6: // SBC
                            addInstruction(alu(GenOpcode::SubtractWithCarry, dstReg, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0x7: // ROR
                            addInstruction(alu(GenOpcode::RotateRight, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeC | writeZ | writeN);
                            break;
                        case 0x8: // TST
                            addInstruction(alu(GenOpcode::And, dstReg, srcReg, GenReg::Temp, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0x9: // NEG
                            addInstruction(loadImm(0));
                            addInstruction(alu(GenOpcode::Subtract, GenReg::Temp, srcReg, dstReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xA: // CMP
                            addInstruction(compare(dstReg, srcReg, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xB: // CMN
                            addInstruction(alu(GenOpcode::Add, dstReg, srcReg, GenReg::Temp, pcSCycles), 2, writeV | writeC | writeZ | writeN);
                            break;
                        case 0xC: // ORR
                            addInstruction(alu(GenOpcode::Or, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0xD: // MUL
                            addInstruction(alu(GenOpcode::Multiply, dstReg, srcReg, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        case 0xE: // BIC
                        {
                            GenOpInfo notOp{};
                            notOp.opcode = GenOpcode::Not;
                            notOp.src[0] = static_cast<uint8_t>(srcReg);
                            notOp.dst[0] = static_cast<uint8_t>(GenReg::Temp);
                            addInstruction(notOp);

                            addInstruction(alu(GenOpcode::And, dstReg, GenReg::Temp, dstReg, pcSCycles), 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }
                        case 0xF: // MVN
                        {
                            GenOpInfo notOp{};
                            notOp.opcode = GenOpcode::Not;
                            notOp.cycles = pcSCycles;
                            notOp.src[0] = static_cast<uint8_t>(srcReg);
                            notOp.dst[0] = static_cast<uint8_t>(dstReg);
                            addInstruction(notOp, 2, preserveV | preserveC | writeZ | writeN);
                            break;
                        }

                        default:
                            printf("unhandled alu op in convertToGeneric %x\n", instOp);
                            done = true;
                    }
                    
                }
                break;
            }

            case 0x5: // formats 7-8
            {
                auto offReg = lowReg((opcode >> 6) & 7);
                auto baseReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                addInstruction(alu(GenOpcode::Add, baseReg, offReg, GenReg::Temp));

                if(opcode & (1 << 9)) // format 8, load/store sign-extended byte/halfword
                {
                    bool hFlag = opcode & (1 << 11);
                    bool signEx = opcode & (1 << 10);

                    if(signEx)
                    {
                        if(hFlag) // LDRSH
                            addInstruction(load(2, GenReg::Temp, dstReg, pcSCycles + 1), 2, GenOp_SignExtend);
                        else // LDRSB
                            addInstruction(load(1, GenReg::Temp, dstReg, pcSCycles + 1), 2, GenOp_SignExtend);
                    }
                    else
                    {
                        if(hFlag) // LDRH
                            addInstruction(load(2, GenReg::Temp, dstReg, pcSCycles + 1), 2);
                        else // STRH
                            addInstruction(store(2, GenReg::Temp, dstReg, pcNCycles), 2, GenOp_UpdateCycles);
                    }
                }
                else // format 7, load/store with reg offset
                {
                    bool isLoad = opcode & (1 << 11);
                    bool isByte = opcode & (1 << 10);

                    if(isLoad)
                        addInstruction(load(isByte ? 1 : 4, GenReg::Temp, dstReg, pcSCycles + 1), 2);
                    else
                        addInstruction(store(isByte ? 1 : 4, GenReg::Temp, dstReg, pcNCycles), 2, GenOp_UpdateCycles);
                }

                break;
            }

            case 0x6: // format 9, load/store with imm offset (words)
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                if(isLoad)
                    loadWithOffset(4, baseReg, offset, dstReg, pcSCycles + 1, 2);
                else
                    storeWithOffset(4, baseReg, offset, dstReg, pcNCycles, 2, GenOp_UpdateCycles);

                break;
            }

            case 0x7: // ... (bytes)
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                if(isLoad)
                    loadWithOffset(1, baseReg, offset, dstReg, pcSCycles + 1, 2);
                else
                    storeWithOffset(1, baseReg, offset, dstReg, pcNCycles, 2, GenOp_UpdateCycles);

                break;
            }

            case 0x8: // format 10, load/store halfword
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = ((opcode >> 6) & 0x1F);
                auto baseReg = lowReg((opcode >> 3) & 7);
                auto dstReg = lowReg(opcode & 7);

                if(isLoad)
                    loadWithOffset(2, baseReg, offset, dstReg, pcSCycles + 1, 2);
                else
                    storeWithOffset(2, baseReg, offset, dstReg, pcNCycles, 2, GenOp_UpdateCycles);

                break;
            }
    
            case 0x9: // format 11, SP-relative load/store
            {
                bool isLoad = opcode & (1 << 11);
                auto offset = opcode & 0xFF;
                auto baseReg = GenReg::R13;
                auto dstReg = lowReg((opcode >> 8) & 7);

                if(isLoad)
                    loadWithOffset(4, baseReg, offset, dstReg, pcSCycles + 1, 2);
                else
                    storeWithOffset(4, baseReg, offset, dstReg, pcNCycles, 2, GenOp_UpdateCycles);

                break;
            }

            case 0xA: // format 12, load address
            {
                bool isSP = opcode & (1 << 11);
                auto dstReg = lowReg((opcode >> 8) & 7);
                auto word = (opcode & 0xFF) << 2;

                if(isSP)
                {
                    addInstruction(loadImm(word));
                    addInstruction(alu(GenOpcode::Add, GenReg::R13, GenReg::Temp, dstReg, pcSCycles), 2);
                }
                else // PC
                {
                    addInstruction(loadImm(((pc + 2) & ~2) + word));
                    addInstruction(move(GenReg::Temp, dstReg, pcSCycles), 2);
                }

                break;
            }

            case 0xB: // misc
            {
                switch((opcode >> 8) & 0xF)
                {
                    case 0x0: // add/sub imm to SP
                    {
                        bool isNeg = opcode & (1 << 7);
                        int off = (opcode & 0x7F) << 2;

                        addInstruction(loadImm(off));

                        if(isNeg)
                            addInstruction(alu(GenOpcode::Subtract, GenReg::R13, GenReg::Temp, GenReg::R13, pcSCycles), 2);
                        else
                            addInstruction(alu(GenOpcode::Add, GenReg::R13, GenReg::Temp, GenReg::R13, pcSCycles), 2);
                        break;
                    }
                    
                    case 0x2:
                    {
                        auto srcReg = lowReg((opcode >> 3) & 7);
                        auto dstReg = lowReg(opcode & 7);

                        switch((opcode >> 6) & 3)
                        {
                            case 0x0: // SXTH
                            case 0x1: // SXTB
                            {
                                GenOpInfo extOp{};
                                extOp.opcode = ((opcode >> 6) & 3) == 0 ? GenOpcode::SignExtend16 : GenOpcode::SignExtend8;
                                extOp.cycles = pcSCycles;
                                extOp.src[0] = static_cast<uint8_t>(srcReg);
                                extOp.dst[0] = static_cast<uint8_t>(dstReg);
                                addInstruction(extOp, 2);
                                break;
                            }
                            case 0x2: // UXTH
                                addInstruction(loadImm(0xFFFF));
                                addInstruction(alu(GenOpcode::And, srcReg, GenReg::Temp, dstReg, pcSCycles), 2);
                                break;
                            case 0x3: // UXTB
                                addInstruction(loadImm(0xFF));
                                addInstruction(alu(GenOpcode::And, srcReg, GenReg::Temp, dstReg, pcSCycles), 2);
                                break;
                        }

                        break;
                    }

                    case 0x4: // PUSH
                    case 0x5:
                    case 0xC: // POP
                    case 0xD:
                    {
                        bool isLoad = opcode & (1 << 11);
                        bool pclr = opcode & (1 << 8); // store LR/load PC
                        uint8_t regList = opcode & 0xFF;
                        auto baseReg = GenReg::R13;

                        if(isLoad) // POP
                        {
                            uint32_t offset = 0;
                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                {
                                    // load
                                    loadWithOffset(4, baseReg, offset, lowReg(i), 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign);
                                    offset++;
                                }
                            }

                            if(pclr)
                            {
                                // load pc (to temp)
                                loadWithOffset(4, baseReg, offset, GenReg::Temp2, 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign);

                                // clear thumb bit
                                addInstruction(loadImm(~1u));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));
        
                                offset++;

                                if(pc > maxBranch)
                                    done = true;
                            }

                            // update SP
                            addInstruction(loadImm(offset * 4));
                            addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg, pclr ? 0 : pcSCycles + 1), pclr ? 0 : 2);

                            if(pclr)
                                addInstruction(jump(GenCondition::Always, GenReg::Temp2, pcSCycles + 1), 2); // TODO: cycles for branch (not implemented in CPU either)
                        }
                        else
                        {
                            uint32_t offset = 0;

                            if(pclr)
                                offset += 4;

                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                    offset += 4;
                            }

                            // update SP
                            addInstruction(loadImm(offset));
                            addInstruction(alu(GenOpcode::Subtract, baseReg, GenReg::Temp, baseReg));

                            offset = 0;
                            for(int i = 0; i < 8; i++)
                            {
                                if(regList & (1 << i))
                                {
                                    // store
                                    bool last = !pclr && (regList >> i) == 1;
                                    storeWithOffset(4, baseReg, offset, lowReg(i), 0, 0, (offset ? GenOp_Sequential : 0) | (last ? GenOp_UpdateCycles : 0));
                                    offset++;
                                }
                            }

                            if(pclr) // store LR
                                storeWithOffset(4, baseReg, offset, GenReg::R14, 0, 0, (offset ? GenOp_Sequential : 0) | GenOp_UpdateCycles);

                            genBlock.instructions.back().cycles = pcNCycles;
                            genBlock.instructions.back().len = 2;
                        }
                        break;   
                    }
                    
                    case 0x6: // CPS
                    {
                        assert((opcode & 0xFFEF) == 0xB662);
                        // assume privileged
                        if(opcode & (1 << 4))
                            addInstruction(loadImm(1));
                        else
                            addInstruction(loadImm(0));

                        addInstruction(move(GenReg::Temp, GenReg::PriMask, pcSCycles), 2);
                        break;
                    }

                    case 0xF: // hints
                    {
                        auto opA = (opcode >> 4) & 0xF;
                        auto opB = opcode & 0xF;

                        if(opB == 0)
                        {
                            switch(opA)
                            {
                                case 0: // NOP
                                {
                                    GenOpInfo op{};
                                    op.opcode = GenOpcode::NOP;
                                    op.cycles = pcSCycles;

                                    addInstruction(op, 2);
                                    break;
                                }

                                default:
                                    done = true;
                                    printf("unhandled hint op in convertToGeneric %X %X\n", opA, opB);
                            }
                        }
                        else
                        {
                            done = true;
                            printf("unhandled hint op in convertToGeneric %X %X\n", opA, opB);
                        }
                        break;
                    }

                    default:
                        done = true;
                        printf("unhandled op in convertToGeneric %04X\n", opcode & 0xFF00);
                }

                break;
            }

            case 0xC: // format 15, multiple load/store
            {
                bool isLoad = opcode & (1 << 11);
                int baseRegIndex = (opcode >> 8) & 7;
                auto baseReg = lowReg(baseRegIndex);
                uint8_t regList = opcode & 0xFF;

                if(!regList)
                {
                    // not quite invalid yet (would fault on v7)
                    if(isLoad)
                    {
                        // load PC
                        addInstruction(load(4, baseReg, GenReg::Temp2, 0));

                        // clear thumb bit
                        addInstruction(loadImm(~1u));
                        addInstruction(alu(GenOpcode::And, GenReg::Temp2, GenReg::Temp, GenReg::Temp2));

                        if(pc > maxBranch)
                            done = true;

                        addInstruction(jump(GenCondition::Always, GenReg::Temp2, pcSCycles + 1), 2); // TODO: cycles for branch (not implemented in CPU either)
                    }
                    else
                    {
                        // store LR
                        addInstruction(store(4, baseReg, GenReg::R14, 0), 0, GenOp_UpdateCycles);

                        // update base
                        addInstruction(loadImm(4));
                        addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg, pcSCycles), 2);
                    }
                }
                else
                {
                    int regCount = 0;
                    int lastRegIndex = 0;
                    for(int i = 0; i < 8; i++)
                    {
                        if(regList & (1 << i))
                        {
                            regCount++;
                            lastRegIndex = i;
                        }
                    }
                   
                    bool first = true;
                    bool wroteBack = false;

                    // prevent overriding base for loads
                    // "A LDM will always overwrite the updated base if the base is in the list."
                    if(isLoad && (regList & (1 << baseRegIndex)))
                    {
                        first = false;

                        // move the base to the last loaded reg so it doesn't get overwritten until we're done
                        auto lastReg = lowReg(lastRegIndex);
                        if(lastReg != baseReg)
                        {
                            addInstruction(move(baseReg, lastReg));
                            baseReg = lastReg;
                        }
                    }

                    if(!isLoad && (regList & (1 << baseRegIndex)))
                    {
                        // on v6, base doesn't get overriden if it's in the list for STM
                        first = false;
                    }

                    uint32_t offset = 0;

                    for(int i = 0; i < 8; i++)
                    {
                        if(!(regList & (1 << i)))
                            continue;

                        bool last = (regList >> i) == 1;

                        auto reg = lowReg(i);
                        if(offset)
                        {
                            if(wroteBack)
                            {
                                // we updated it so index backwards
                                addInstruction(loadImm(regCount * 4 - offset));
                                addInstruction(alu(GenOpcode::Subtract, baseReg, GenReg::Temp, GenReg::Temp));
                            }
                            else
                            {
                                addInstruction(loadImm(offset));
                                addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, GenReg::Temp));
                            }
                        }

                        int flags = (offset ? GenOp_Sequential : 0) | GenOp_ForceAlign;
                        if(isLoad)
                            addInstruction(load(4, offset ? GenReg::Temp : baseReg, reg, 0), 0, flags);
                        else
                            addInstruction(store(4, offset ? GenReg::Temp : baseReg, reg, 0), 0, flags | (last ? GenOp_UpdateCycles : 0));

                        // base write-back is on the second cycle of the instruction
                        // which is when the first reg is written
                        if(first)
                        {
                            addInstruction(loadImm(regCount * 4));
                            addInstruction(alu(GenOpcode::Add, baseReg, GenReg::Temp, baseReg));
                            wroteBack = true; // unfortunately, we're still using it to index
                        }

                        first = false;

                        offset += 4;
                    }

                    genBlock.instructions.back().cycles = isLoad ? (pcSCycles + 1) : pcNCycles;
                    genBlock.instructions.back().len = 2;
                }

                break;
            }

            case 0xD: // formats 16-17, conditional branch + SWI
            {
                auto cond = (opcode >> 8) & 0xF;
                int offset = static_cast<int8_t>(opcode & 0xFF);

                if(cond == 15)
                {
                    printf("unhandled SWI in convertToGeneric %04X\n", opcode & 0xF000);
                    done = true;
                }
                else
                {
                    auto genCond = static_cast<GenCondition>(cond); // they happen to match
                    auto addr = pc + 2 + offset * 2;
                    addInstruction(loadImm(addr, pcSCycles));
                    addInstruction(jump(genCond, GenReg::Temp, pcSCycles + pcNCycles), 2);

                    updateEnd(addr);
                }

                break;
            }

            case 0xE: // format 18, unconditional branch
            {
                uint32_t offset = static_cast<int16_t>(opcode << 5) >> 4; // sign extend and * 2
                addInstruction(loadImm(pc + 2 + offset));
                addInstruction(jump(GenCondition::Always, GenReg::Temp, pcSCycles * 2 + pcNCycles), 2);

                if(pc > maxBranch)
                    done = true;
                break;
            }

            case 0xF: // 32-bit encoding
            {
                uint32_t opcode32 = opcode << 16 | *pcPtr++;
                pc += 2;

                // decode
                if((opcode32 & 0x18008000) != 0x10008000)
                {
                    done = true;
                    break;
                }

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

                    // LR = PC | 1
                    addInstruction(loadImm(pc | 1));
                    addInstruction(move(GenReg::Temp, GenReg::R14));

                    // jump
                    addInstruction(loadImm(pc + offset));
                    addInstruction(jump(GenCondition::Always, GenReg::Temp, pcNCycles + pcSCycles * 3), 4, GenOp_Call);
                }
                else if(op2 & 0b101)
                {
                    printf("unhandled op in convertToGeneric %08X\n", opcode32 & 0xF7F0F000);
                    done = true;
                }
                else
                {
                    switch(op1)
                    {
                        case 0x38: // MSR
                        case 0x39:
                        {
                            auto srcReg = reg((opcode32 >> 16) & 0xF);
                            auto sysm = opcode32 & 0xFF;

                            if((sysm >> 3) == 0)
                            {
                                // APSR
                                addInstruction(loadImm(~0xF8000000));
                                addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::CPSR));
                                addInstruction(loadImm(0xF8000000));
                                addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                                addInstruction(alu(GenOpcode::Or, GenReg::CPSR, GenReg::Temp, GenReg::CPSR, pcSCycles * 2 + 1), 4);
                            }
                            else if((sysm >> 3) == 1)
                            {
                                // MSP/PSP
                                // assuming priviledged
                                if(sysm == 8) // MSP
                                {
                                    addInstruction(loadImm(~3));
                                    addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                                    addInstruction(move(GenReg::Temp, GenReg::R13, pcSCycles * 2 + 1), 4);
                                }
                                else if(sysm == 9)
                                {
                                    printf("unhandled MSR PSP in convertToGeneric\n");
                                    done = true;
                                }
                            }
                            else if((sysm >> 3) == 2)
                            {
                                // PRIMASK/CONTROL
                                // assuming priviledged

                                if(sysm == 0x10) // PRIMASK
                                {
                                    addInstruction(loadImm(1));
                                    addInstruction(alu(GenOpcode::And, GenReg::Temp, srcReg, GenReg::Temp));
                                    addInstruction(move(GenReg::Temp, GenReg::PriMask, pcSCycles * 2 + 1), 4);
                                }
                                else
                                {
                                    printf("unhandled MSR CONTROL in convertToGeneric\n");    
                                    done = true;
                                }
                            }
                            break;
                        }

                        case 0x3B: // misc
                        {
                            auto op = (opcode32 >> 4) & 0xF;

                            if(op == 0x4 || op == 0x5) // DSB/DMB
                            {
                                GenOpInfo op{};
                                op.opcode = GenOpcode::NOP;
                                op.cycles = pcSCycles * 2 + 1;

                                addInstruction(op, 4);
                            }
                            else
                                printf("unhandled misc ctrl op in convertToGeneric %X\n", op);

                            break;
                        }

                        case 0x3E: // MRS
                        case 0x3F:
                        {
                            auto dstReg = reg((opcode32 >> 8) & 0xF);
                            auto sysm = opcode32 & 0xFF;

                            if((sysm >> 3) == 0)
                            {
                                // xPSR
                                uint32_t mask = 0;
                                if(sysm & 1) // IPSR
                                    mask |= 0x1FF;

                                // if(sysm & 2) // T bit reads as 0 so do nothing

                                if((sysm & 4) == 0) // APSR
                                    mask |= 0xF8000000;

                                // psr & mask
                                addInstruction(loadImm(mask));
                                addInstruction(alu(GenOpcode::And, GenReg::CPSR, GenReg::Temp, GenReg::Temp));
                                // separate mov to help the target
                                addInstruction(move(GenReg::Temp, dstReg, pcSCycles * 2 + 1), 4);

                            }
                            else if((sysm >> 3) == 2)
                            {
                                // PRIMASK/CONTROL
                                if(sysm == 0x10)
                                {
                                    // primask & 1
                                    addInstruction(loadImm(1));
                                    addInstruction(alu(GenOpcode::And, GenReg::PriMask, GenReg::Temp, GenReg::Temp));
                                    addInstruction(move(GenReg::Temp, dstReg, pcSCycles * 2 + 1), 4);
                                }
                                else if(sysm == 0x14)
                                {
                                    // control & 3
                                    addInstruction(loadImm(3));
                                    addInstruction(alu(GenOpcode::And, GenReg::Control, GenReg::Temp, GenReg::Temp));
                                    addInstruction(move(GenReg::Temp, dstReg, pcSCycles * 2 + 1), 4);
                                }
                            }
                            else
                            {
                                printf("unhandled MRS in convertToGeneric %02X\n", sysm);
                                done = true;
                            }
                            break;
                        }

                        default:
                            printf("unhandled op in convertToGeneric %08X\n", opcode32 & 0xF7F0F000);
                            done = true;
                    }
                }

                break;
            }

            default:
            {
                printf("invalid op in convertToGeneric %Xxxx\n", opcode >> 12);
                done = true;
            }
        }
    }
}

void ARMv6MRecompiler::compileEntry()
{
    auto entryPtr = target.compileEntry(curCodePtr, codeBufSize);
    entryFunc = reinterpret_cast<CompiledFunc>(entryPtr);
}

// wrappers around member funcs
uint8_t ARMv6MRecompiler::readMem8(ARMv6MCore *cpu, uint32_t addr, int &cycles, uint8_t flags)
{
    syncClockForPeriphAccess(cpu, addr);

    return cpu->readMem8(addr, cycles, flags & (GenOp_Sequential >> 8));
}

uint32_t ARMv6MRecompiler::readMem16(ARMv6MCore *cpu, uint32_t addr, int &cycles, uint8_t flags)
{
    syncClockForPeriphAccess(cpu, addr);

    auto ret = cpu->readMem16(addr, cycles, flags & (GenOp_Sequential >> 8));

    // unaligned sign extend half -> byte
    if((addr & 1) && (flags & (GenOp_SignExtend >> 8)) && (ret & 0x80))
        ret |= 0xFF00;

    return ret;
}

uint32_t ARMv6MRecompiler::readMem32(ARMv6MCore *cpu, uint32_t addr, int &cycles, uint8_t flags)
{
    if(flags & (GenOp_ForceAlign >> 8))
        addr &= ~3;

    syncClockForPeriphAccess(cpu, addr);

    return cpu->readMem32(addr, cycles, flags & (GenOp_Sequential >> 8));
}

int ARMv6MRecompiler::writeMem8(ARMv6MCore *cpu, uint32_t addr, uint8_t data, int &cycles, uint8_t flags, int cyclesToRun)
{
    invalidateCode(cpu, addr);
    syncClockForPeriphAccess(cpu, addr);

    cpu->writeMem8(addr, data, cycles, flags & (GenOp_Sequential >> 8));
    return updateCyclesForWrite(cpu, cyclesToRun);
}

int ARMv6MRecompiler::writeMem16(ARMv6MCore *cpu, uint32_t addr, uint16_t data, int &cycles, uint8_t flags, int cyclesToRun)
{
    invalidateCode(cpu, addr);
    syncClockForPeriphAccess(cpu, addr);

    cpu->writeMem16(addr, data, cycles, flags & (GenOp_Sequential >> 8));
    return updateCyclesForWrite(cpu, cyclesToRun);
}

int ARMv6MRecompiler::writeMem32(ARMv6MCore *cpu, uint32_t addr, uint32_t data, int &cycles, uint8_t flags, int cyclesToRun)
{
    invalidateCode(cpu, addr);
    syncClockForPeriphAccess(cpu, addr);

    cpu->writeMem32(addr, data, cycles, flags & (GenOp_Sequential >> 8));
    return updateCyclesForWrite(cpu, cyclesToRun);
}

void ARMv6MRecompiler::invalidateCode(ARMv6MCore *cpu, uint32_t addr)
{
    auto &compiler = cpu->compiler;

    int index = addr >= 0x20040000;
    
    if(addr < compiler.minRAMCode[index] || addr >= compiler.maxRAMCode[index])
        return;

    auto end = compiler.compiled.end();

    bool erased = false;

    for(auto it = compiler.ramStartIt[index]; it != end;)
    {
        if(addr >= it->first && addr < it->second.endPC)
        {
#ifdef RECOMPILER_DEBUG
            printf("invalidate compiled code @%08X in %08X-%08X\n", addr, it->first, it->second.endPC);
#endif
            // rewind if last code compiled
            // TODO: reclaim memory in other cases
            if(it->second.endPtr == compiler.curCodePtr)
                compiler.curCodePtr = it->second.startPtr;

            // invalidate any saved return addresses
            for(auto &saved : compiler.savedExits)
            {
                if(std::get<1>(saved) >= it->first && std::get<1>(saved) <= it->second.endPC)
                    saved = {nullptr, 0, 0};
            }

            it = compiler.compiled.erase(it);
            erased = true;

            continue; // might have compiled the same code more than once
        }

        // past this address, stop
        if(it->first > addr)
            break;

        ++it;
    }

    if(erased)
    {
        compiler.ramStartIt[0] = compiler.compiled.lower_bound(0x20000000);
        compiler.ramStartIt[1] = compiler.compiled.lower_bound(0x20040000);

        if(compiler.ramStartIt[index] != compiler.compiled.end())
        {
            // can update min easily
            compiler.minRAMCode[index] = compiler.ramStartIt[index]->first;
        }
        else
        {
            // there's no code left, reset the ranges
            compiler.minRAMCode[index] = 0xFFFFFFFF;
            compiler.maxRAMCode[index] = 0;
        }
    }
}

void ARMv6MRecompiler::syncClockForPeriphAccess(ARMv6MCore *cpu, uint32_t addr)
{
    // peripherals heavily rely on the cpu clock being up-to-date for sync
    if(addr >= 0x40000000)
    {
        cpu->getClock().addCycles(cpu->compiler.cycleCount);
        // keep track for updating cyclesExecuted later
        cpu->compiler.extraCycleCount += cpu->compiler.cycleCount;
        cpu->compiler.cycleCount = 0;
    }
}

int ARMv6MRecompiler::updateCyclesForWrite(ARMv6MCore *cpu, int cyclesToRun)
{
    if(!(cpu->primask & 1) && cpu->needException)
        return 0;

    auto cyclesToIntr = cpu->clock.getCyclesToTime(cpu->mem.getNextInterruptTime());

    return std::min(cyclesToRun, static_cast<int>(cyclesToIntr - cpu->compiler.cycleCount));
}
