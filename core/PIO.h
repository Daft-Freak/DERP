#pragma once

#include <cstdint>
#include <functional>

#include "hardware/structs/pio.h"

#include "ClockTarget.h"
#include "FIFO.h"

class MemoryBus;

class PIO final
{
public:
    using UpdateCallback = std::function<void(uint64_t, PIO &pio)>;

    PIO(MemoryBus &mem, int index);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(hw.inte0 || hw.inte1)
            update(target);
    }

    void setUpdateCallback(UpdateCallback cb);

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

    const pio_hw_t &getHW(){return hw;}
    FIFO<uint32_t, 4> &getRXFIFO(int i) {return rxFifo[i];}
    FIFO<uint32_t, 4> &getTXFIFO(int i) {return txFifo[i];}

    void updateFifoStatus(int sm);

private:
    MemoryBus &mem;
    int index;

    ClockTarget clock;

    pio_hw_t hw;

    UpdateCallback updateCallback;

    // TODO: joined
    FIFO<uint32_t, 4> rxFifo[NUM_PIO_STATE_MACHINES];
    FIFO<uint32_t, 4> txFifo[NUM_PIO_STATE_MACHINES];
};
