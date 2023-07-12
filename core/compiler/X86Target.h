#pragma once

#include <map>
#include <optional>
#include <variant>

#include "RecompilerGeneric.h"

enum class Reg8;
enum class Reg16;
enum class Reg32;
enum class Reg64;

class X86Builder;

class X86Target final
{
public:
    X86Target(){}

    void init(SourceInfo sourceInfo, void *cpuPtr);

    const SourceInfo &getSourceInfo() {return sourceInfo;}

    bool compile(uint8_t *&codePtr, uint8_t *codeBufEnd, uint32_t pc, GenBlockInfo &blockInfo);

    uint8_t *compileEntry(uint8_t *&codeBuf, unsigned int codeBufSize);

private:
    std::optional<Reg8> mapReg8(uint8_t index);
    std::optional<Reg16> mapReg16(uint8_t index);
    std::optional<Reg32> mapReg32(uint8_t index);
    std::optional<Reg64> mapReg64(uint8_t index);

    SourceFlagInfo &getFlagInfo(SourceFlagType flag);
    uint8_t flagWriteMask(SourceFlagType flag);
    bool writesFlag(uint16_t opFlags, SourceFlagType flag);

    bool needStackAlign() const;

    bool isCallSaved(Reg16 reg) const;
    bool isCallSaved(Reg8 reg) const;

    int callSaveIndex(Reg16 reg) const;
    int callSaveIndex(Reg8 reg) const;

    void callSaveIfNeeded(X86Builder &builder, int &saveState) const;

    void callRestore(X86Builder &builder, int &saveState, int toIndex) const;
    void callRestore(X86Builder &builder, Reg8 dstReg, bool zeroExtend, bool signExtend) const;
    void callRestoreIfNeeded(X86Builder &builder, int &saveState) const;
    void callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg16, uint16_t> val, int &saveState) const;
    void callRestoreIfNeeded(X86Builder &builder, std::variant<std::monostate, Reg8, uint8_t> val, int &saveState) const;

    SourceInfo sourceInfo;
    void *cpuPtr;

    std::map<uint8_t, Reg32> regAlloc;

    uint8_t flagsReg = 0;
    uint8_t flagsSize = 0;
    uint32_t flagsMask = 0;
    uint8_t flagMap[6]; // map from SourceFlagType to flags bit

    uint8_t *exitPtr = nullptr, *saveAndExitPtr = nullptr, *exitForCallPtr = nullptr;

    // everything we might call
    uint8_t *readMem8Ptr = nullptr, *readMem16Ptr = nullptr, *readMem32Ptr = nullptr;
    uint8_t *writeMem8Ptr = nullptr, *writeMem16Ptr = nullptr, *writeMem32Ptr = nullptr;

    uint8_t numSavedRegs;
    bool saveR15;
};