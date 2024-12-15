#include <cassert>
#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/dma.h"
#include "hardware/regs/dreq.h"
#include "hardware/regs/intctrl.h"

#include "DMA.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::DMA;

DMA::DMA(MemoryBus &mem) : mem(mem)
{
}

void DMA::reset()
{
    clock.reset();

    curChannel = 0;

    for(auto &reg : readAddr)
        reg = DMA_CH0_READ_ADDR_RESET;

    for(auto &reg : writeAddr)
        reg = DMA_CH0_WRITE_ADDR_RESET;

    for(auto &reg : transferCount)
        reg = DMA_CH0_TRANS_COUNT_RESET;

    for(auto &reg : transferCountReload)
        reg = 0;

    for(auto &reg : ctrl)
        reg = DMA_CH0_CTRL_TRIG_RESET;

    for(auto &t : transfersInProgress)
        t = 0;

    for(auto &d : dreqCounter)
        d = 0;

    interrupts = 0;
    interruptEnables[0] = interruptEnables[1] = 0;

    channelTriggered = 0;
    treqCounterMask = 0;
    activeTREQMask = 0;
}

void DMA::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);

    auto readChannel = curReadChannel;
    auto writeChannel = curWriteChannel;

    // TODO: priority, ring, chaining, sniff...

    for(uint32_t cycle = 0; cycle < passed; cycle++)
    {
        if(!(channelTriggered & treqCounterMask) && writeAddressFifo.empty())
        {
            clock.addCycles(passed - cycle);
            break;
        }

        // transfer write data
        if(writeChannel >= 0)
        {
            assert(!transferDataFifo.empty());

            int cycles; // TODO
            uint32_t val = transferDataFifo.pop();

            (this->*writeFuncs[writeChannel])(val, cycles);

            transfersInProgress[writeChannel]--;
            
            // decrement count/finish
            if(--transferCount[writeChannel] == 0)
            {
                channelTriggered &= ~(1 << writeChannel); // done

                int treq = (ctrl[writeChannel] & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
                if(treq != DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT)
                    activeTREQMask &= ~(1 << treq);

                interrupts |= (1 << writeChannel);
            
                if(interruptEnables[0] & (1 << writeChannel))
                    mem.setPendingIRQ(DMA_IRQ_0);
                if(interruptEnables[1] & (1 << writeChannel))
                    mem.setPendingIRQ(DMA_IRQ_1);
            }
    
            writeChannel = -1;
        }

        // set write address
        if(writeChannel == -1 && !transferDataFifo.empty())
        {
            assert(!writeAddressFifo.empty());

            writeChannel = writeChannelFifo.pop();
            curWriteAddr = writeAddressFifo.pop();
        }

        // transfer read data
        if(readChannel >= 0 && !transferDataFifo.full())
        {
            int cycles; // TODO
            uint32_t val = (this->*readFuncs[readChannel])(cycles);
            transferDataFifo.push(val);
            readChannel = -1;
        }

        // set read address
        if(readChannel == -1 && !readAddressFifo.empty() && !transferDataFifo.full()/*?*/)
        {
            readChannel = readChannelFifo.pop();
            curReadAddr = readAddressFifo.pop();
        }

        // address generation
        if(!writeAddressFifo.full())
        {
            // read/write addr are generated at the same time and write happens last, so read should also have room
            assert(!readAddressFifo.full());

            for(int i = 0; i < numChannels; i++, curChannel++)
            {
                if(curChannel == numChannels)
                    curChannel = 0;

                if(!(channelTriggered & (1 << curChannel)) || !(ctrl[curChannel] & DMA_CH0_CTRL_TRIG_EN_BITS))
                    continue;

                // check DREQ
                if(!dreqCounter[curChannel])
                    continue;

                // don't queue any more if enough transfers in progress to complete
                if(transferCount[curChannel] <= transfersInProgress[curChannel])
                    continue;

                transfersInProgress[curChannel]++;

                // decrement DREQs if not permanent
                if(dreqCounter[curChannel] != 0xFF)
                {
                    if(--dreqCounter[curChannel] == 0)
                        treqCounterMask &= ~(1 << curChannel);
                }

                readAddressFifo.push(readAddr[curChannel]);
                writeAddressFifo.push(writeAddr[curChannel]);

                readChannelFifo.push(curChannel);
                writeChannelFifo.push(curChannel);

                // increment
                int transferSize = 1 << ((ctrl[curChannel] & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);

                if(ctrl[curChannel] & DMA_CH0_CTRL_TRIG_INCR_READ_BITS)
                    readAddr[curChannel] += transferSize;

                if(ctrl[curChannel] & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS)
                    writeAddr[curChannel] += transferSize;

                break;
            }
        }

        clock.addCycles(1);
    }

    curReadChannel = readChannel;
    curWriteChannel = writeChannel;
}

uint64_t DMA::getNextInterruptTime(uint64_t target)
{
    auto anyInterruptEnable = interruptEnables[0] | interruptEnables[1];
    if(!channelTriggered || !anyInterruptEnable)
        return target;

    for(int i = 0; i < numChannels; i++)
    {
        if(!(channelTriggered & (1 << i)) || !(ctrl[i] & DMA_CH0_CTRL_TRIG_EN_BITS))
            continue;

        if(!(anyInterruptEnable & (1 << i)))
            continue;

        // this assumes a single channel
        // but being early is okay (except for perf)
        auto cycles = estimateChannelCompletion(i);

        auto time = clock.getTimeToCycles(cycles);

        if(time < target)
            target = time;
    }

    return target;
}

uint32_t DMA::regRead(uint64_t time, uint32_t addr)
{
    if(addr < DMA_INTR_OFFSET)
    {
        int ch = addr / 0x40;
        assert(ch < numChannels);

        if(channelTriggered & (1 << ch))
            mem.syncDMA(time);

        switch(addr & 0x3F)
        {
            case DMA_CH0_READ_ADDR_OFFSET:
            case DMA_CH0_AL1_READ_ADDR_OFFSET:
            case DMA_CH0_AL2_READ_ADDR_OFFSET:
            case DMA_CH0_AL3_READ_ADDR_TRIG_OFFSET:
                return readAddr[ch];
            case DMA_CH0_WRITE_ADDR_OFFSET:
            case DMA_CH0_AL1_WRITE_ADDR_OFFSET:
            case DMA_CH0_AL2_WRITE_ADDR_TRIG_OFFSET:
            case DMA_CH0_AL3_WRITE_ADDR_OFFSET:
                return writeAddr[ch];
            case DMA_CH0_TRANS_COUNT_OFFSET:
            case DMA_CH0_AL1_TRANS_COUNT_TRIG_OFFSET:
            case DMA_CH0_AL2_TRANS_COUNT_OFFSET:
            case DMA_CH0_AL3_TRANS_COUNT_OFFSET:
                return transferCount[ch];
            case DMA_CH0_CTRL_TRIG_OFFSET:
            case DMA_CH0_AL1_CTRL_OFFSET:
            case DMA_CH0_AL2_CTRL_OFFSET:
            case DMA_CH0_AL3_CTRL_OFFSET:
                return ctrl[ch] | (channelTriggered & (1 << ch) ? DMA_CH0_CTRL_TRIG_BUSY_BITS : 0);
        }
    }
    else
    {
        mem.syncDMA(time); // technically not needed for INTE or INTF

        switch(addr)
        {
            case DMA_INTR_OFFSET:
                return interrupts;
            case DMA_INTE0_OFFSET:
                return interruptEnables[0];
            case DMA_INTS0_OFFSET:
                return interrupts & interruptEnables[0]; // TODO: force
            case DMA_INTE1_OFFSET:
                return interruptEnables[1];
            case DMA_INTS1_OFFSET:
                return interrupts & interruptEnables[1]; // TODO: force
        }
        logf(LogLevel::NotImplemented, logComponent, "R %08X", addr);
    }
    return 0xBADADD55;
}

void DMA::regWrite(uint64_t time, uint32_t addr, uint32_t data)
{
    update(time);

    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < DMA_INTR_OFFSET)
    {
        int ch = addr / 0x40;
        assert(ch < numChannels);

        switch(addr & 0x3F)
        {
            case DMA_CH0_READ_ADDR_OFFSET:
            case DMA_CH0_AL1_READ_ADDR_OFFSET:
            case DMA_CH0_AL2_READ_ADDR_OFFSET:
            case DMA_CH0_AL3_READ_ADDR_TRIG_OFFSET:
                updateReg(readAddr[ch], data, atomic);
                break;
            case DMA_CH0_WRITE_ADDR_OFFSET:
            case DMA_CH0_AL1_WRITE_ADDR_OFFSET:
            case DMA_CH0_AL2_WRITE_ADDR_TRIG_OFFSET:
            case DMA_CH0_AL3_WRITE_ADDR_OFFSET:
                updateReg(writeAddr[ch], data, atomic);
                break;
            case DMA_CH0_TRANS_COUNT_OFFSET:
            case DMA_CH0_AL1_TRANS_COUNT_TRIG_OFFSET:
            case DMA_CH0_AL2_TRANS_COUNT_OFFSET:
            case DMA_CH0_AL3_TRANS_COUNT_OFFSET:
                updateReg(transferCountReload[ch], data, atomic);
                break;
            case DMA_CH0_CTRL_TRIG_OFFSET:
            case DMA_CH0_AL1_CTRL_OFFSET:
            case DMA_CH0_AL2_CTRL_OFFSET:
            case DMA_CH0_AL3_CTRL_OFFSET:
                updateReg(ctrl[ch], data & ~(1 << 24 | 1 << 31), atomic);
                updateReadFunc(ch);
                updateWriteFunc(ch);
                break;
        }

        if((addr & 0xF) == 0xC && (ctrl[ch] & DMA_CH0_CTRL_TRIG_EN_BITS)) // *_TRIG
        {
            transferCount[ch] = transferCountReload[ch];
            channelTriggered |= 1 << ch;

            int treq = (ctrl[ch] & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
            if(treq != DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT)
                activeTREQMask |= 1 << treq;

            dreqHandshake(time, ch);
        }
    }
    else
    {
        switch(addr)
        {
            case DMA_INTE0_OFFSET:
                updateReg(interruptEnables[0], data, atomic);
                return;
            case DMA_INTS0_OFFSET:
            case DMA_INTS1_OFFSET:
            {
                if(atomic == 0)
                {
                    interrupts &= ~data;
                    return;
                }
                else
                {
                    // DMA atomic does an actual read
                    // anything using this is probably buggy
                    // https://github.com/raspberrypi/pico-sdk/issues/974

                    auto oldVal = regRead(time, addr);
                    updateReg(oldVal, data, atomic);
                    interrupts &= ~oldVal;
                    return;
                }
                break;
            }
            case DMA_INTE1_OFFSET:
                updateReg(interruptEnables[1], data, atomic);
                return;
        }

        logf(LogLevel::NotImplemented, logComponent, "W %03X%s%08X", addr, op[atomic], data);
    }
}

bool DMA::isChannelActive(int ch) const
{
    return channelTriggered & (1 << ch);
}

void DMA::triggerDREQ(uint64_t time, int dreq)
{
    if(!(activeTREQMask & (1ull << dreq)))
        return;

    // TODO: sync other periphs?
    update(time);

    // TODO: faster lookup
    for(int i = 0; i < numChannels; i++)
    {
        int treq = (ctrl[i] & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;

        if(treq == dreq)
        {
            dreqCounter[i] = dreqCounter[i] == 0x3F ? 0x3F : (dreqCounter[i] + 1);
            treqCounterMask |= (1 << i);
        }
    }
}

uint64_t DMA::getActiveTREQMask() const
{
    return activeTREQMask;
}

template<class T, bool bswap>
uint32_t DMA::doRead(int &cycles)
{
    uint32_t val = mem.read<T>(this, curReadAddr, cycles);

    if(bswap)
    {
        if(sizeof(T) == 2)
            val = val >> 8 | val << 8;
        else if(sizeof(T) == 4)
            val = val >> 24 | val << 24 | (val & 0xFF00) << 8 | (val & 0xFF0000) >> 8;
    }

    return val;
}

template<class T>
void DMA::doWrite(uint32_t val, int &cycles)
{
    mem.write<T>(this, curWriteAddr, val, cycles);
}

void DMA::updateReadFunc(int channel)
{
    int transferSize = ((ctrl[channel] & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
    bool bswap = ctrl[channel] & DMA_CH0_CTRL_TRIG_BSWAP_BITS;

    if(transferSize == 0)
        readFuncs[channel] = bswap ? &DMA::doRead<uint8_t, true> : &DMA::doRead<uint8_t, false>;
    else if(transferSize == 1)
        readFuncs[channel] = bswap ? &DMA::doRead<uint16_t, true> : &DMA::doRead<uint16_t, false>;
    else
        readFuncs[channel] = bswap ? &DMA::doRead<uint32_t, true> : &DMA::doRead<uint32_t, false>;
}

void DMA::updateWriteFunc(int channel)
{
    int transferSize = ((ctrl[channel] & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);

    if(transferSize == 0)
        writeFuncs[channel] = &DMA::doWrite<uint8_t>;
    else if(transferSize == 1)
        writeFuncs[channel] = &DMA::doWrite<uint16_t>;
    else
        writeFuncs[channel] = &DMA::doWrite<uint32_t>;
}

void DMA::dreqHandshake(uint64_t time, int channel)
{
    int treq = (ctrl[channel] & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    
    dreqCounter[channel] = 0;
    treqCounterMask &= ~(1 << channel);

    switch(treq)
    {
        case DREQ_PIO0_TX0:
        case DREQ_PIO0_TX1:
        case DREQ_PIO0_TX2:
        case DREQ_PIO0_TX3:
        case DREQ_PIO0_RX0:
        case DREQ_PIO0_RX1:
        case DREQ_PIO0_RX2:
        case DREQ_PIO0_RX3:
            mem.getPIO(0).dreqHandshake(time, treq);
            break;

        case DREQ_PIO1_TX0:
        case DREQ_PIO1_TX1:
        case DREQ_PIO1_TX2:
        case DREQ_PIO1_TX3:
        case DREQ_PIO1_RX0:
        case DREQ_PIO1_RX1:
        case DREQ_PIO1_RX2:
        case DREQ_PIO1_RX3:
            mem.getPIO(1).dreqHandshake(time, treq);
            break;

        case DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT:
            dreqCounter[channel] = 0xFF; // special value
            treqCounterMask |= (1 << channel);
            break;

        default:
            logf(LogLevel::NotImplemented, logComponent, "dma ch %i TREQ %i", channel, treq);
    }
}

uint32_t DMA::estimateChannelCompletion(int channel) const
{
    auto ret = transferCount[channel];

    int treq = (ctrl[channel] & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
    
    // attempt to take into account the peripheral generating the DREQ
    // being VERY optimistic...

    switch(treq)
    {
        case DREQ_PIO0_TX0:
        case DREQ_PIO0_TX1:
        case DREQ_PIO0_TX2:
        case DREQ_PIO0_TX3:
        {
            int sm = (treq - DREQ_PIO0_TX0);
            auto clkdiv = mem.getPIO(0).getHW().sm[sm].clkdiv >> 8;

            auto tmp = uint64_t(ret) * clkdiv;

            // get estimated time between pulls from PIO
            // should be best-case
            auto estPullTime = mem.getPIO(0).getMinCyclesBetweenPulls(sm);
            if(estPullTime)
                tmp *= estPullTime;
            
            ret = tmp >> 8;
            break;
        }
        case DREQ_PIO0_RX0:
        case DREQ_PIO0_RX1:
        case DREQ_PIO0_RX2:
        case DREQ_PIO0_RX3:
        {
            // can't do much better than scaling by clkdiv
            // unless we try analysing the program...
            int sm = treq - DREQ_PIO0_RX0;
            auto clkdiv = mem.getPIO(0).getHW().sm[sm].clkdiv >> 8;
            
            ret = (ret * clkdiv) >> 8;
            break;
        }
        case DREQ_PIO1_TX0:
        case DREQ_PIO1_TX1:
        case DREQ_PIO1_TX2:
        case DREQ_PIO1_TX3:
        {
            int sm = (treq - DREQ_PIO1_TX0);
            auto clkdiv = mem.getPIO(1).getHW().sm[sm].clkdiv >> 8;

            auto tmp = uint64_t(ret) * clkdiv;

            auto estPullTime = mem.getPIO(1).getMinCyclesBetweenPulls(sm);
            if(estPullTime)
                tmp *= estPullTime;
            
            ret = tmp >> 8;
            break;
        }
        case DREQ_PIO1_RX0:
        case DREQ_PIO1_RX1:
        case DREQ_PIO1_RX2:
        case DREQ_PIO1_RX3:
        {
            int sm = treq - DREQ_PIO1_RX0;
            auto clkdiv = mem.getPIO(1).getHW().sm[sm].clkdiv >> 8;
            
            ret = (ret * clkdiv) >> 8;
            break;
        }

        case DMA_CH0_CTRL_TRIG_TREQ_SEL_VALUE_PERMANENT:
            break;
    }

    return ret;
}
