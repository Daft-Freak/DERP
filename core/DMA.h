#pragma once

#include <cstdint>

#include "hardware/platform_defs.h"

#include "ClockTarget.h"

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

    int curChannel;

    // not using the structs here, too much repetition+padding
    static const int numChannels = NUM_DMA_CHANNELS;
    uint32_t readAddr[numChannels];
    uint32_t writeAddr[numChannels];
    uint32_t transferCount[numChannels], transferCountReload[numChannels];
    uint32_t ctrl[numChannels];

    uint32_t interrupts;
    uint32_t interruptEnables[2];

    uint32_t channelTriggered;
};