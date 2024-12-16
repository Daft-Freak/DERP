#pragma once

#include <cstdint>
#include <functional>

#include "hardware/structs/pio.h"

#include "ClockTarget.h"
#include "FIFO.h"

class MemoryBus;

class PIO final : public ClockedDevice
{
public:
    using TXCallback = std::function<void(uint64_t, PIO &pio, int, uint32_t)>;

    PIO(MemoryBus &mem, int index);

    void reset();

    void update(uint64_t target);
    bool needUpdateForInterrupts()
    {
        return hw.inte0 || hw.inte1;
    }

    void setSpeedHackEnabled(bool enabled);

    void setTXCallback(TXCallback cb);

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint64_t time, uint32_t addr);
    void regWrite(uint64_t time, uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

    int getDeviceFlags() const {return 0;}

    const pio_hw_t &getHW(){return hw;}
    FIFO<uint32_t, 4> &getRXFIFO(int i) {return rxFifo[i];}
    FIFO<uint32_t, 4> &getTXFIFO(int i) {return txFifo[i];}

    void updateFifoStatus(int sm);

    void dreqHandshake(uint64_t time, int dreq);

    uint32_t getMinCyclesBetweenPulls(int sm) const {return minCyclesBetweenPulls[sm];}
    void resetMinCyclesBetweenPulls(int sm);

private:
    struct Instruction
    {
        uint8_t op;
        uint8_t delay;
        uint8_t sideSet;
        uint8_t params[3];
    };

    enum class ExecResult
    {
        Done,
        Jumped,
        Stalled
    };

    Instruction decodeInstruction(uint16_t op, int sm);

    void analyseProgram(int sm, int modInstrIndex = -1);

    int getDREQNum(int sm, bool isTx) const;

    void updateSM(int sm, uint32_t target, int32_t &cycleOffset);
    ExecResult executeSMInstruction(int sm, const Instruction &instr, uint32_t clockOffset);

    MemoryBus &mem;
    int index;

    ClockTarget clock;

    pio_hw_t hw;

    bool speedHack;
    int speedHackCounter[NUM_PIO_STATE_MACHINES];

    TXCallback txCallback;

    uint32_t clockFrac[NUM_PIO_STATE_MACHINES];

    struct
    {
        uint32_t osr;
        uint32_t isr;

        uint32_t x;
        uint32_t y;

        uint8_t pc;

        uint8_t osc, isc; // counters
    } regs[NUM_PIO_STATE_MACHINES];

    Instruction instrs[NUM_PIO_STATE_MACHINES][PIO_INSTRUCTION_COUNT];

    uint8_t txStall, rxStall; // internal flags

    // TODO: joined
    FIFO<uint32_t, 4> rxFifo[NUM_PIO_STATE_MACHINES];
    FIFO<uint32_t, 4> txFifo[NUM_PIO_STATE_MACHINES];

    // best-case time between pulls
    uint32_t cyclesSinceLastPull[NUM_PIO_STATE_MACHINES];
    uint32_t minCyclesBetweenPulls[NUM_PIO_STATE_MACHINES];

    // info analysed from program
    int cyclesBetweenPulls[NUM_PIO_STATE_MACHINES];
    int failedAnalysisAttempts;
};
