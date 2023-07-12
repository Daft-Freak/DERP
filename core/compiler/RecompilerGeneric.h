#pragma once
#include <cstdint>
#include <vector>

enum class GenOpcode : uint8_t
{
    NOP = 0,
    LoadImm, // always to reg 0
    Move,
    Load,
    Load2,
    Load4,
    Store,
    Store2,
    Store4,

    Add,
    AddWithCarry,
    And,
    Compare,
    Multiply,
    Or,
    Subtract,
    SubtractWithCarry,
    Xor,

    Not,

    RotateLeft,
    RotateLeftCarry,
    RotateRight,
    RotateRightCarry,
    ShiftLeft,
    ShiftRightArith,
    ShiftRightLogic,

    Jump,
};

// 1st src of jump
enum class GenCondition : uint8_t
{
    Equal = 0,
    NotEqual,
    CarrySet,
    CarryClear,
    Negative,
    Positive,
    OverflowSet,
    OverflowClear,
    Higher, // C == 1 && Z == 0
    LowerSame, // C == 0 || Z == 1
    GreaterEqual, // N == V
    LessThan, // N != V
    GreaterThan, // Z == 0 && N == V
    LessThanEqual, // Z == 1 || N != V
    Always
};

enum GenOpFlags
{
    GenOp_PreserveFlags = 0xF,

    GenOp_WriteFlags = 0xF << 4,

    GenOp_Call = 1 << 8, // branch is a call, code will return here later
    GenOp_BranchTarget = 1 << 9,
    GenOp_Exit = 1 << 10,

    GenOp_MagicAlt1 = 1 << 11, // for when an op sometimes has slightly different behaviour
    GenOp_MagicAlt2 = 1 << 12,

    GenOp_Sequential = 1 << 11, // for load/store
    GenOp_SignExtend = 1 << 12, // for load
    GenOp_UpdateCycles = 1 << 12, // for store
    GenOp_ForceAlign = 1 << 13, // for load/store
};

struct GenOpInfo
{
    GenOpcode opcode;
    uint8_t cycles;
    uint8_t len;
    union
    {
        struct
        {
            uint8_t src[2], dst[1];
        };
        uint32_t imm;
    };
    uint16_t flags;
};

/*enum GenBlockFlags
{
};*/

struct GenBlockInfo
{
    std::vector<GenOpInfo> instructions;
    uint32_t flags;
};

enum class SourceRegType : uint8_t
{
    General = 0,
    Flags,
    Temp, // internal
};

struct SourceRegInfo
{
    const char *label;
    uint8_t size; // bits
    SourceRegType type;
    uint8_t alias; // this reg is part of a larger reg
    uint32_t aliasMask;
    uint16_t cpuOffset; // for load/store
};

enum class SourceFlagType : uint8_t
{
    Carry = 0,
    Zero,
    Negative,
    Overflow
};

struct SourceFlagInfo
{
    char label;
    uint8_t bit;
    SourceFlagType type;
};

struct SourceInfo
{
    std::vector<SourceRegInfo> registers;
    std::vector<SourceFlagInfo> flags;

    uint8_t pcSize;
    uint8_t pcPrefetch; // offset to add on exit for prefetch
    uint16_t pcOffset;

    uint8_t cycleMul; // cycles to add for each cycle in an op

    int extraCPUOffsets[5]; // source specific ops

    bool (*shouldSyncForAddress)(uint16_t addr); // return true to sync cycle count before accessing this address
    bool (*shouldSyncForRegIndex)(uint8_t reg, const GenBlockInfo &block);

    uint16_t (*getRegOffset)(void *cpu, uint8_t reg);

    // emulator interface
    bool *exitCallFlag;
    uint8_t **savedExitPtr;

    uint32_t *cycleCount;

    uint8_t (*readMem8)(void *cpu, uint32_t addr, int &cycles, uint8_t flags);
    uint32_t (*readMem16)(void *cpu, uint32_t addr, int &cycles, uint8_t flags); // returning 32-bit here is deliberate
    uint32_t (*readMem32)(void *cpu, uint32_t addr, int &cycles, uint8_t flags);

    int (*writeMem8)(void *cpu, uint32_t addr, uint8_t data, int &cycles, uint8_t flags, int cyclesToRun);
    int (*writeMem16)(void *cpu, uint32_t addr, uint16_t data, int &cycles, uint8_t flags, int cyclesToRun);
    int (*writeMem32)(void *cpu, uint32_t addr, uint32_t data, int &cycles, uint8_t flags, int cyclesToRun);
};

void analyseGenBlock(uint32_t pc, uint32_t endPC, GenBlockInfo &blockInfo, const SourceInfo &sourceInfo);

void printGenBlock(uint32_t pc, const GenBlockInfo &block, const SourceInfo &sourceInfo);