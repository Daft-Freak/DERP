#pragma once
#include <cstdint>

enum class Reg8
{
    AL = 0,
    CL,
    DL,
    BL,
    AH,
    CH,
    DH,
    BH,

    R8B,
    R9B,
    R10B,
    R11B,
    R12B,
    R13B,
    R14B,
    R15B,

    // these replace AH-DH with a REX prefix
    SPL = 4 | 0x10,
    BPL,
    SIL,
    DIL
};

enum class Reg16
{
    AX = 0,
    CX,
    DX,
    BX,
    SP,
    BP,
    SI,
    DI,

    R8W,
    R9W,
    R10W,
    R11W,
    R12W,
    R13W,
    R14W,
    R15W
};

enum class Reg32
{
    EAX = 0,
    ECX,
    EDX,
    EBX,
    ESP,
    EBP,
    ESI,
    EDI,

    R8D,
    R9D,
    R10D,
    R11D,
    R12D,
    R13D,
    R14D,
    R15D
};

enum class Reg64
{
    RAX = 0,
    RCX,
    RDX,
    RBX,
    RSP,
    RBP,
    RSI,
    RDI,

    R8,
    R9,
    R10,
    R11,
    R12,
    R13,
    R14,
    R15
};

enum class Condition
{
    O = 0,
    NO,
    B,
    AE,
    E,
    NE,
    BE,
    A,
    S,
    NS,
    P,
    NP,
    L,
    GE,
    LE,
    G
};


class X86Builder
{
public:
    class RMOperand
    {
    public:
        explicit constexpr RMOperand(Reg64 reg) : base(reg), w(4) {} // register
        explicit constexpr RMOperand(Reg32 reg) : base(static_cast<Reg64>(reg)), w(3) {} // register
        explicit constexpr RMOperand(Reg16 reg) : base(static_cast<Reg64>(reg)), w(2) {} // register
        explicit constexpr RMOperand(Reg8 reg) : base(static_cast<Reg64>(reg)), w(1) {} // register
        constexpr RMOperand(Reg64 base, int disp) : base(base), disp(disp) {} // memory
        constexpr RMOperand(Reg64 base, Reg64 index, int disp = 0, int scale = 1) : base(base), index(index), scale(scale), disp(disp) // memory, with index/scale
        {
            assert(scale == 1 || scale == 2 || scale == 4 || scale == 8);
        }

        constexpr bool isMem() const {return w == 0;}

        constexpr Reg32 getReg32() const {return static_cast<Reg32>(base);}

        Reg64 base, index = Reg64::RSP;
        uint8_t w = 0; // 0 = indirect, 1 = 8, 2 = 16, ...
        uint8_t scale = 0;
        int disp = 0;
    };

    X86Builder(uint8_t *ptr, uint8_t *endPtr) : ptr(ptr), endPtr(endPtr){}

    void add(Reg32 dst, Reg32 src);
    void add(Reg16 dst, Reg16 src);
    void add(Reg8 dst, Reg8 src);
    void add(RMOperand dst, Reg32 src);
    void add(Reg32 dst, uint32_t imm);
    void add(Reg8 dst, uint8_t imm);
    void add(RMOperand dst, uint32_t imm);
    void add(Reg64 dst, int8_t imm);
    void add(Reg32 dst, int8_t imm);
    void add(Reg16 dst, int8_t imm);
    void addD(RMOperand dst, int8_t imm);
    void addW(RMOperand dst, int8_t imm);

    void adc(Reg32 dst, Reg32 src);
    void adc(Reg8 dst, Reg8 src);
    void adc(Reg8 dst, uint8_t imm);

    void and_(Reg32 dst, Reg32 src);
    void and_(Reg16 dst, Reg16 src);
    void and_(Reg8 dst, Reg8 src);
    void and_(Reg32 dst, uint32_t imm);
    void and_(Reg8 dst, uint8_t imm);
    void and_(Reg32 dst, int8_t imm);

    void btr(Reg32 base, uint8_t off);

    void call(int disp);
    void call(Reg64 r);

    void cmc();

    void cmp(Reg32 dst, Reg32 src);
    void cmp(Reg8 dst, Reg8 src);
    void cmp(Reg32 dst, uint32_t imm);
    void cmp(Reg8 dst, uint8_t imm);
    void cmp(Reg32 dst, int8_t imm);
    void cmp(RMOperand dst, uint8_t imm);

    void dec(Reg16 r);
    void dec(Reg8 r);

    void imul(Reg32 dst, Reg32 src);

    void inc(Reg16 r);
    void inc(Reg8 r);

    void jcc(Condition cc, int disp);

    void jmp(int disp, bool forceLong = false);
    void jmp(Reg64 r);

    void lea(Reg64 dst, RMOperand m);
    void lea(Reg32 dst, RMOperand m);

    void mov(Reg64 dst, Reg64 src);
    void mov(Reg32 dst, Reg32 src);
    void mov(Reg16 dst, Reg16 src);
    void mov(Reg8 dst, Reg8 src);
    void mov(RMOperand dst, Reg64 src);
    void mov(RMOperand dst, Reg32 src);
    void mov(RMOperand dst, Reg16 src);
    void mov(RMOperand dst, Reg8 src);
    void mov(Reg64 dst, RMOperand src);
    void mov(Reg32 dst, RMOperand src);
    void mov(Reg16 dst, RMOperand src);
    void mov(Reg8 dst, RMOperand src);
    void mov(RMOperand dst, Reg32 src, int w); // internal
    void mov(Reg32 dst, RMOperand src, int w); // internal
    void mov(Reg64 r, uint64_t imm);
    void mov(Reg32 r, uint32_t imm);
    void mov(Reg8 r, uint8_t imm);
    void mov(RMOperand dst, uint32_t imm);
    void mov(RMOperand dst, uint8_t imm);

    void movsx(Reg32 dst, Reg16 src);
    void movsx(Reg32 dst, Reg8 src);

    void movzx(Reg32 dst, Reg16 src);
    void movzx(Reg32 dst, Reg8 src);
    void movzxW(Reg32 dst, RMOperand src);

    void not_(Reg32 r);
    void not_(Reg8 r);

    void or_(Reg32 dst, Reg32 src);
    void or_(Reg16 dst, Reg16 src);
    void or_(Reg8 dst, Reg8 src);
    void or_(Reg32 dst, uint32_t imm);
    void or_(Reg8 dst, uint8_t imm);

    void pop(Reg64 r);

    void push(Reg64 r);

    void rclCL(Reg8 dst);
    void rcl(Reg8 r, uint8_t count);

    void rcrCL(Reg8 dst);
    void rcr(Reg8 r, uint8_t count);

    void ret();

    void rolCL(Reg8 dst);
    void rol(Reg8 r, uint8_t count);

    void rorCL(Reg32 dst);
    void rorCL(Reg8 dst);
    void ror(Reg32 r, uint8_t count);
    void ror(Reg8 r, uint8_t count);

    void sarCL(Reg32 dst);
    void sarCL(Reg8 dst);
    void sar(Reg32 r, uint8_t count);
    void sar(Reg8 r, uint8_t count);

    void sbb(Reg32 dst, Reg32 src);
    void sbb(Reg8 dst, Reg8 src);
    void sbb(Reg8 dst, uint8_t imm);

    void setcc(Condition cc, Reg8 dst);

    void shrCL(Reg32 dst);
    void shrCL(Reg8 dst);
    void shr(Reg32 r, uint8_t count);
    void shr(Reg8 r, uint8_t count);

    void shlCL(Reg32 dst);
    void shlCL(Reg8 dst);
    void shl(Reg32 r, uint8_t count);
    void shl(Reg8 r, uint8_t count);

    void stc();

    void sub(Reg32 dst, Reg32 src);
    void sub(Reg16 dst, Reg16 src);
    void sub(Reg8 dst, Reg8 src);
    void sub(RMOperand dst, Reg32 src);
    void sub(Reg32 dst, uint32_t imm);
    void sub(Reg8 dst, uint8_t imm);
    void sub(RMOperand dst, uint32_t src);
    void sub(Reg64 dst, int8_t imm);
    void sub(Reg32 dst, int8_t imm);
    void subD(RMOperand dst, int8_t imm);

    void test(Reg32 dst, uint32_t imm);
    void test(Reg8 dst, uint8_t imm);

    void xchg(Reg32 dst, Reg32 src);
    void xchg(Reg8 dst, Reg8 src);

    void xor_(Reg32 dst, Reg32 src);
    void xor_(Reg8 dst, Reg8 src);
    void xor_(Reg32 dst, uint32_t imm);
    void xor_(Reg8 dst, uint8_t imm);

    uint8_t *getPtr() const {return ptr;}

    void resetPtr(uint8_t *oldPtr);

    void patch(uint8_t *patchPtr, uint8_t *patchEndPtr);
    void endPatch();

    void removeRange(uint8_t *startPtr, uint8_t *endPtr);

    bool getError() const {return error;}

private:
    void write(uint8_t b);

    void encodeModRM(int reg1, int reg2Op = 0); // mod 3
    void encodeModRM(RMOperand rm, int reg2Op = 0);
    void encodeModRMReg8(int reg1, int reg2); // mod 3 (extra validation fot 8bit regs)
    void encodeREX(bool w, int reg, int index, int base);
    void encodeREX(bool w, int reg, RMOperand rm);

    uint8_t *ptr, *endPtr;
    uint8_t *savedPtr, *savedEndPtr;

    bool error = false; // super basic error handling
};
