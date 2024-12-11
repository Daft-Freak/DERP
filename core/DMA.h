#pragma once

#include <cstdint>

#include "hardware/platform_defs.h"

#include "ClockTarget.h"
#include "FIFO.h"

class MemoryBus;

class DMA final : public ClockedDevice
{
public:
    DMA(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    bool needUpdateForInterrupts()
    {
        return channelTriggered & (interruptEnables[0] | interruptEnables[1]);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint64_t time, uint32_t addr);
    void regWrite(uint64_t time, uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

    bool isChannelActive(int ch) const;

    void triggerDREQ(uint64_t time, int dreq);
    uint64_t getActiveTREQMask() const;

    int getDeviceFlags() const {return 0;}

private:
    using ReadFunc = uint32_t(DMA::*)(int &);
    using WriteFunc = void(DMA::*)(uint32_t, int &);

    template<class T, bool bswap>
    uint32_t doRead(int &cycles);

    template<class T>
    void doWrite(uint32_t val, int &cycles);

    void updateReadFunc(int channel);
    void updateWriteFunc(int channel);

    void dreqHandshake(uint64_t time, int channel);

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
    uint8_t dreqCounter[numChannels]; // 6 bit?

    ReadFunc readFuncs[numChannels];
    WriteFunc writeFuncs[numChannels];

    uint32_t interrupts;
    uint32_t interruptEnables[2];

    uint32_t channelTriggered;
    uint32_t treqCounterMask; // if counter > 0
    uint64_t activeTREQMask; // TREQs on currently triggered channels
};