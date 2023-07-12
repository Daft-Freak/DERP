#include <cstdio>

#include "RecompilerGeneric.h"

static int getFlagsForCond(GenCondition cond, int carryMask, int zeroMask, int negMask, int overflowMask)
{
    switch(cond)
    {
        case GenCondition::Equal:
        case GenCondition::NotEqual:
            return zeroMask;

        case GenCondition::CarryClear:
        case GenCondition::CarrySet:
            return carryMask;
        
        case GenCondition::Negative:
        case GenCondition::Positive:
            return negMask;

        case GenCondition::OverflowSet:
        case GenCondition::OverflowClear:
            return overflowMask;

        case GenCondition::Higher:
        case GenCondition::LowerSame:
            return carryMask | zeroMask;

        case GenCondition::GreaterEqual:
        case GenCondition::LessThan:
            return negMask | overflowMask;
    
        case GenCondition::GreaterThan:
        case GenCondition::LessThanEqual:
            return zeroMask | negMask | overflowMask;

        case GenCondition::Always:
            break;
    }

    return 0;
}

void analyseGenBlock(uint32_t pc, uint32_t endPC, GenBlockInfo &blockInfo, const SourceInfo &sourceInfo)
{
    auto startPC = pc;

    // find flag masks/reg
    int carryMask = 0, zeroMask = 0, negMask = 0, overflowMask = 0;
    uint8_t flagsReg = 0xFF;

    int i = 0;
    for(auto &flag : sourceInfo.flags)
    {
        if(flag.type == SourceFlagType::Carry)
            carryMask = 1 << i;
        else if(flag.type == SourceFlagType::Zero)
            zeroMask = 1 << i;
        else if(flag.type == SourceFlagType::Negative)
            negMask = 1 << i;
        else if(flag.type == SourceFlagType::Overflow)
            overflowMask = 1 << i;

        i++;
    }

    for(auto it = sourceInfo.registers.begin() + 1; it != sourceInfo.registers.end(); ++it)
    {
        if(it->type == SourceRegType::Flags)
            flagsReg = it - sourceInfo.registers.begin();
    }

    // TODO: we end up scanning for branch targets a lot
    auto findBranchTarget = [&blockInfo](uint32_t pc, uint32_t target, std::vector<GenOpInfo>::iterator it)
    {
        auto searchPC = pc;
        if(target < pc)
        {
            auto prevIt = std::make_reverse_iterator(it + 1);
    
            for(; prevIt != blockInfo.instructions.rend() && searchPC >= target; ++prevIt)
            {
                searchPC -= prevIt->len;
                if(searchPC == target)
                {
                    auto ret = prevIt.base() - 1;
                    // need to go to the start of the instruction
                    // (the op after the last one to increment PC)
                    while(ret != blockInfo.instructions.begin() && !(ret - 1)->len)
                        --ret;

                    return ret;
                }
            }
        }
        else
        {
            for(auto nextIt = it + 1; nextIt != blockInfo.instructions.end() && searchPC <= target; searchPC += nextIt->len, ++nextIt)
            {
                if(searchPC == target)
                    return nextIt;
            }
        }

        return blockInfo.instructions.end();
    };

    auto getReadFlags = [zeroMask, carryMask, negMask, overflowMask, flagsReg](GenOpInfo &instr)
    {
        if(instr.opcode == GenOpcode::AddWithCarry || instr.opcode == GenOpcode::SubtractWithCarry || instr.opcode == GenOpcode::RotateLeftCarry || instr.opcode == GenOpcode::RotateRightCarry)
            return carryMask;

        // flags from condition
        if(instr.opcode == GenOpcode::Jump)
        {
            auto cond = static_cast<GenCondition>(instr.src[0]);
            return getFlagsForCond(cond, carryMask, zeroMask, negMask, overflowMask);
        }

        // reads flags register directly
        if(instr.opcode != GenOpcode::LoadImm && (instr.src[0] == flagsReg || instr.src[1] == flagsReg))
            return 0xF; // all

        return 0;
    };

    auto getWrittenFlags = [flagsReg](GenOpInfo &instr) -> int
    {
        // special case for loading flags, writes all of them
        if(instr.opcode == GenOpcode::Load && instr.dst[0] == flagsReg)
            return GenOp_WriteFlags;

        // if the write and preserve flag are both set, it might not set the flag depending on the inputs
        return instr.flags & GenOp_WriteFlags & ~((instr.flags & GenOp_PreserveFlags) << 4);
    };

    std::vector<uint8_t> origFlags; // there aren't enough bits in ->flags...
    origFlags.resize(blockInfo.instructions.size());

    for(auto it = blockInfo.instructions.begin(); it != blockInfo.instructions.end(); ++it)
    {
        auto &instr = *it;
        pc += instr.len;

        if(instr.flags & GenOp_WriteFlags)
        {
            // find actually used flags
            int read = 0;

            int falseBranchFlags = 0; // flags used by the "other" branch
            bool inBranch = false;

            // save flags
            origFlags[it - blockInfo.instructions.begin()] = instr.flags & GenOp_WriteFlags;

            // look ahead until we have no flags wrtiten that are not read
            auto nextPC = pc;
            for(auto next = it + 1; next != blockInfo.instructions.end() && (instr.flags & GenOp_WriteFlags) != read << 4;)
            {
                nextPC += next->len;

                // collect read flags
                read |= getReadFlags(*next);
                read &= (instr.flags & GenOp_WriteFlags) >> 4; // can't have a read for a flag we're not setting

                if(next->opcode == GenOpcode::Jump)
                {
                    // don't go too deep
                    if(inBranch)
                        break;

                    inBranch = true;

                    bool isConditional = static_cast<GenCondition>(next->src[0]) != GenCondition::Always;

                    // make sure we know the target addr
                    if(next->src[1] != 0 || (next - 1)->opcode != GenOpcode::LoadImm)
                        break;

                    auto target = (next - 1)->imm;

                    auto targetInstr = findBranchTarget(nextPC, target, next);

                    // bad branch, give up
                    if(targetInstr == blockInfo.instructions.end())
                        break;

                    if(!isConditional)
                    {
                        // follow it
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                    else
                    {
                        // follow false branch until we hit another branch
                        falseBranchFlags = instr.flags & GenOp_WriteFlags;
                        for(auto falseInstr = next + 1; falseInstr != blockInfo.instructions.end() && falseBranchFlags != read << 4; ++falseInstr)
                        {
                            if(falseInstr->opcode == GenOpcode::Jump)
                                break;

                            read |= getReadFlags(*falseInstr) & ((instr.flags & GenOp_WriteFlags) >> 4);
                            falseBranchFlags = (falseBranchFlags & ~getWrittenFlags(*falseInstr)) | read << 4;
                        }

                        // follow the true branch
                        next = targetInstr;
                        nextPC = target;
                        continue;
                    }
                }

                auto written = getWrittenFlags(*next);
                if(next < it) // backwards jump, restore old flags
                    written = origFlags[next - blockInfo.instructions.begin()];

                // clear overriden flags (keep any that are used)
                instr.flags = (instr.flags & ~(written)) | read << 4 | falseBranchFlags;

                ++next;
            }
        }

        // jumps with immediate addr may be branches
        if(instr.opcode == GenOpcode::Jump && instr.src[1] == 0 && (it - 1)->opcode == GenOpcode::LoadImm)
        {
            // get target from LoadImm
            auto target = (it - 1)->imm;

            // not reachable, mark as exit
            if(target < startPC || target >= endPC)
                instr.flags |= GenOp_Exit;
            else
            {
                // find and mark target
                auto targetInstr = findBranchTarget(pc, target, it);

                if(targetInstr != blockInfo.instructions.end())
                    targetInstr->flags |= GenOp_BranchTarget;
                else
                {
                    // failed to find target
                    // may happen if there's a jump over some data
                    instr.flags |= GenOp_Exit;
                }
            }
        }
        else if(instr.opcode == GenOpcode::Jump)
            instr.flags |= GenOp_Exit; // we don't know where it goes
    }
}

void printGenBlock(uint32_t pc, const GenBlockInfo &block, const SourceInfo &sourceInfo)
{
    struct OpMeta
    {
        const char *name;
        uint8_t numSrc, numDst;
    };

    static const OpMeta opMeta[]
    {
        {"nop ", 0, 0},
        {"ldim", 0, 0},
        {"move", 1, 1},
        {"ld1 ", 1, 1},
        {"ld2 ", 1, 1},
        {"ld4 ", 1, 1},
        {"str1", 2, 0},
        {"str2", 2, 0},
        {"str4", 2, 0},
        {"add ", 2, 1},
        {"addc", 2, 1},
        {"and ", 2, 1},
        {"comp", 2, 0},
        {"mult", 2, 1},
        {"or  ", 2, 1},
        {"sub ", 2, 1},
        {"subc", 2, 1},
        {"xor ", 2, 1},
        {"not ", 1, 1},
        {"rol ", 2, 1},
        {"rolc", 2, 1},
        {"ror ", 2, 1},
        {"rorc", 2, 1},
        {"shl ", 2, 1},
        {"shra", 2, 1},
        {"shrl", 2, 1},
        {"jump", 2, 0},
    };

    static const OpMeta specialOpMeta[]
    {
        {"_   ", 0, 0},
    };

    static const char *condName[]
    {
        "equ",
        "neq",
        "cas",
        "cac",
        "neg",
        "pos",
        "ovs",
        "ovc",
        "hi ",
        "los",
        "gte",
        "lt ",
        "gt ",
        "lte",
        "alw",
    };

    // flag info
    char flagChars[]{'X', 'X', 'X', 'X'};

    int carryMask = 0, zeroMask = 0, negMask = 0, overflowMask = 0;

    int i = 0;
    for(auto &flag : sourceInfo.flags)
    {
        if(flag.type == SourceFlagType::Carry)
            carryMask = 1 << i;
        else if(flag.type == SourceFlagType::Zero)
            zeroMask = 1 << i;
        else if(flag.type == SourceFlagType::Negative)
            negMask = 1 << i;
        else if(flag.type == SourceFlagType::Overflow)
            overflowMask = 1 << i;

        flagChars[i++] = flag.label;
    }
    

    uint32_t lastPC = 1;

    for(auto &instr : block.instructions)
    {
        if(lastPC != pc)
        {
            lastPC = pc;
            printf("%08X: ", pc);
        }
        else
            printf("        : ");

        auto intOp = static_cast<int>(instr.opcode);

        auto &meta = intOp & 0x80 ? specialOpMeta[intOp & 0x7F] : opMeta[intOp];

        if(instr.opcode == GenOpcode::NOP && !instr.cycles)
        {
            printf("UNHANDLED\n");
            pc += instr.len;
            continue;
        }

        printf("%s", meta.name);

        if(instr.opcode == GenOpcode::LoadImm)
            printf(" %08X      ", instr.imm);
        else
        {
            // src
            int i = 0;
            
            // first jump src is the condition
            if(instr.opcode == GenOpcode::Jump)
            {
                printf(" %s", condName[instr.src[0]]);
                i++;
            }

            for(; i < meta.numSrc; i++)
            {
                if(instr.src[i] < sourceInfo.registers.size())
                    printf(" %s", sourceInfo.registers[instr.src[i]].label);
                else
                    printf(" R%02i", instr.src[i]);
            }

            // align
            for(; i < 2; i++)
                printf("    ");

            if(meta.numDst)
                printf(" ->");
            else
                printf("   ");
    
            // dst
            for(i = 0; i < meta.numDst; i++)
            {
                if(instr.dst[i] < sourceInfo.registers.size())
                    printf(" %s", sourceInfo.registers[instr.dst[i]].label);
                else
                    printf(" R%02i", instr.dst[i]);
            }
            
            // align
            for(; i < 1; i++)
                printf("    ");
        }

        // flags

        // work out flags read
        uint8_t flagsRead = 0;
        if(instr.opcode == GenOpcode::AddWithCarry || instr.opcode == GenOpcode::SubtractWithCarry || instr.opcode == GenOpcode::RotateLeftCarry || instr.opcode == GenOpcode::RotateRightCarry)
            flagsRead = carryMask;
        else if(instr.opcode == GenOpcode::Jump)
        {
            auto cond = static_cast<GenCondition>(instr.src[0]);
            flagsRead = getFlagsForCond(cond, carryMask, zeroMask, negMask, overflowMask);
        }
        // else anything that reads a flags register directly

        if((instr.flags & 0xFF) || flagsRead)
        {
            printf(" flags %c%c%c%c -> %c%c%c%c", 
                    flagsRead   & 0x01 ? flagChars[0] : '-',
                    flagsRead   & 0x02 ? flagChars[1] : '-',
                    flagsRead   & 0x04 ? flagChars[2] : '-',
                    flagsRead   & 0x08 ? flagChars[3] : '-',
                    instr.flags & 0x10 ? flagChars[0] : '-',
                    instr.flags & 0x20 ? flagChars[1] : '-',
                    instr.flags & 0x40 ? flagChars[2] : '-',
                    instr.flags & 0x80 ? flagChars[3] : '-');
        }
        else
            printf("                   ");

        // cycle count
        if(instr.cycles)
            printf(" (%i cycles)", instr.cycles);
        else
            printf("           ");

        // branches
        // TODO: more info?
        if(instr.flags & GenOp_Exit)
            printf(" ex");

        if(instr.flags & GenOp_BranchTarget)
            printf(" bt");

        printf("\n");


        pc += instr.len;
    }
}