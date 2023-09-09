#include <cassert>
#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/dma.h"
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

    interrupts = 0;
    interruptEnables[0] = interruptEnables[1] = 0;

    channelTriggered = 0;
}

void DMA::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);


    // TODO: DREQ, priority, ring, chaining, sniff...

    for(uint32_t cycle = 0; cycle < passed; cycle++)
    {
        if(!channelTriggered && writeAddressFifo.empty())
        {
            clock.addCycles(passed - cycle);
            break;
        }

        // transfer write data
        if(curWriteChannel >= 0)
        {
            assert(!transferDataFifo.empty());

            int transferSize = ((ctrl[curWriteChannel] & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);

            int cycles; // TODO
            uint32_t val = transferDataFifo.pop();

            if(transferSize == 0)
                mem.write<uint8_t>(this, curWriteAddr, val, cycles);
            else if(transferSize == 1)
                mem.write<uint16_t>(this, curWriteAddr, val, cycles);
            else
                mem.write<uint32_t>(this, curWriteAddr, val, cycles);

            transfersInProgress[curWriteChannel]--;
            
            // decrement count/finish
            if(--transferCount[curWriteChannel] == 0)
            {
                channelTriggered &= ~(1 << curWriteChannel); // done

                interrupts |= (1 << curWriteChannel);
            
                if(interruptEnables[0] & (1 << curWriteChannel))
                    mem.setPendingIRQ(DMA_IRQ_0);
                if(interruptEnables[1] & (1 << curWriteChannel))
                    mem.setPendingIRQ(DMA_IRQ_1);
            }
    
            curWriteChannel = -1;
        }

        // set write address
        if(!transferDataFifo.empty() && curWriteChannel == -1)
        {
            assert(!writeAddressFifo.empty());

            curWriteChannel = writeChannelFifo.pop();
            curWriteAddr = writeAddressFifo.pop();
        }

        // transfer read data
        if(!transferDataFifo.full() && curReadChannel >= 0)
        {
            int transferSize = ((ctrl[curReadChannel] & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB);
            bool bswap = ctrl[curReadChannel] & DMA_CH0_CTRL_TRIG_BSWAP_BITS;
            
            int cycles; // TODO
            uint32_t val;

            if(transferSize == 0)
                val = mem.read<uint8_t>(this, curReadAddr, cycles);
            else if(transferSize == 1)
            {
                val = mem.read<uint16_t>(this, curReadAddr, cycles);
                if(bswap)
                    val = val >> 8 | val << 8;
            }
            else
            {
                val = mem.read<uint32_t>(this, curReadAddr, cycles);
                if(bswap)
                    val = val >> 24 | val << 24 | (val & 0xFF00) << 8 | (val & 0xFF0000) >> 8;
            }

            transferDataFifo.push(val);
            curReadChannel = -1;
        }

        // set read address
        if(!readAddressFifo.empty() && !transferDataFifo.full()/*?*/ && curReadChannel == -1)
        {
            curReadChannel = readChannelFifo.pop();
            curReadAddr = readAddressFifo.pop();
        }

        // address generation
        if(!readAddressFifo.full() && !writeAddressFifo.full())
        {
            for(int i = 0; i < numChannels; i++, curChannel++)
            {
                if(curChannel == numChannels)
                    curChannel = 0;

                if(!(channelTriggered & (1 << curChannel)) || !(ctrl[curChannel] & DMA_CH0_CTRL_TRIG_EN_BITS))
                    continue;

                // don't queue any more if enough transfers in progress to complete
                if(transferCount[curChannel] <= transfersInProgress[curChannel])
                    continue;

                transfersInProgress[curChannel]++;

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

        // FIXME: this is very wrong and doesn't even try... but it shouldn't be too late
        auto cycles = transferCount[i];

        auto time = clock.getTimeToCycles(cycles);

        if(time < target)
            target = time;
    }

    return target;
}

uint32_t DMA::regRead(uint32_t addr)
{
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

void DMA::regWrite(uint32_t addr, uint32_t data)
{
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
                break;
        }

        if((addr & 0xF) == 0xC && (ctrl[ch] & DMA_CH0_CTRL_TRIG_EN_BITS)) // *_TRIG
        {
            transferCount[ch] = transferCountReload[ch];
            channelTriggered |= 1 << ch;
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

                    auto oldVal = regRead(addr);
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