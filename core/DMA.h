#pragma once

#include <cstdint>

#include "hardware/platform_defs.h"

#include "ClockTarget.h"
#include "FIFO.h"

class MemoryBus;

class DMA final
{
public:
    DMA(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(channelTriggered & (interruptEnables[0] | interruptEnables[1]))
            update(target);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

    bool isChannelActive(int ch) const;

private:
    MemoryBus &mem;

    ClockTarget clock;

    // for address generator
    int curChannel;

    // sizes are a guess
    FIFO<uint32_t, 4> readAddressFifo;
    FIFO<uint32_t, 4> writeAddressFifo;
    FIFO<uint32_t, 4> transferDataFifo;

    // need to know which channel a read/write is for to get the ctrl and count
    FIFO<int, 4> readChannelFifo;
    FIFO<int, 4> writeChannelFifo;

    uint32_t curReadAddr, curWriteAddr;
    int curReadChannel = -1, curWriteChannel = -1;

    // not using the structs here, too much repetition+padding
    static const int numChannels = NUM_DMA_CHANNELS;
    uint32_t readAddr[numChannels];
    uint32_t writeAddr[numChannels];
    uint32_t transferCount[numChannels], transferCountReload[numChannels];
    uint32_t ctrl[numChannels];

    uint32_t transfersInProgress[numChannels];

    uint32_t interrupts;
    uint32_t interruptEnables[2];

    uint32_t channelTriggered;
};