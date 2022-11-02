#include <cassert>
#include <cstdio>

#include "DMA.h"

#include "MemoryBus.h"

DMA::DMA(MemoryBus &mem) : mem(mem)
{
}

void DMA::reset()
{
    clock.reset();

    curChannel = 0;

    for(auto &reg : readAddr)
        reg = 0;

    for(auto &reg : writeAddr)
        reg = 0;

    for(auto &reg : transferCount)
        reg = 0;

    for(auto &reg : transferCountReload)
        reg = 0;

    for(auto &reg : ctrl)
        reg = 0;

    interrupts = 0;
    interruptEnables[0] = interruptEnables[1] = 0;

    channelTriggered = 0;
}

void DMA::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);


    // TODO: DREQ, FIFOs, priority, ring, chaining, bswap, sniff...

    for(uint32_t cycle = 0; cycle < passed; cycle++)
    {
        if(!channelTriggered)
            break;

        for(int i = 0; i < numChannels; i++, curChannel++)
        {
            if(curChannel == numChannels)
                curChannel = 0;

            if(!(channelTriggered & (1 << curChannel)) || !(ctrl[curChannel] & 1/*EN*/))
                continue;

            int transferSize = 1 << ((ctrl[curChannel] >> 2) & 3);

            int cycles; // TODO
            if(transferSize == 1)
                mem.write(this, writeAddr[curChannel], mem.read<uint8_t>(this, readAddr[curChannel], cycles, false), cycles, false);
            else if(transferSize == 2)
                mem.write(this, writeAddr[curChannel], mem.read<uint16_t>(this, readAddr[curChannel], cycles, false), cycles, false);
            else
                mem.write(this, writeAddr[curChannel], mem.read<uint32_t>(this, readAddr[curChannel], cycles, false), cycles, false);

            if(ctrl[curChannel] & (1 << 4)/*INCR_READ*/)
                readAddr[curChannel] += transferSize;

            if(ctrl[curChannel] & (1 << 5)/*INCR_WRITE*/)
                writeAddr[curChannel] += transferSize;

            if(--transferCount[curChannel] == 0)
            {
                channelTriggered &= ~(1 << curChannel); // done

                interrupts |= (1 << curChannel);
            
                if(interruptEnables[0] & (1 << curChannel))
                    mem.setPendingIRQ(11); // DMA_IRQ_0
                if(interruptEnables[1] & (1 << curChannel))
                    mem.setPendingIRQ(12); // DMA_IRQ_1
            }
            break;
        }
    }

    clock.addCycles(passed);
}

void DMA::updateForInterrupts(uint64_t target)
{
    if(channelTriggered & (interruptEnables[0] | interruptEnables[1]))
        update(target);
}

uint32_t DMA::regRead(uint32_t addr)
{
    if(addr < 0x400)
    {
        int ch = addr / 0x40;
        assert(ch < numChannels);

        switch(addr & 0x3F)
        {
            case 0x00: // READ_ADDR
            case 0x14: // AL1_READ_ADDR
            case 0x28: // AL2_READ_ADDR
            case 0x3C: // AL3_READ_ADDR_TRIG
                return readAddr[ch];
            case 0x04: // WRITE_ADDR
            case 0x18: // AL1_WRITE_ADDR
            case 0x2C: // AL2_WRITE_ADDR_TRIG
            case 0x34: // AL3_WRITE_ADDR
                return writeAddr[ch];
            case 0x08: // TRANS_COUNT
            case 0x1C: // AL1_TRANS_COUNT_TRIG
            case 0x24: // AL2_TRANS_COUNT
            case 0x38: // AL3_TRANS_COUNT
                return transferCount[ch];
            case 0x0C: // CTRL_TRIG
            case 0x10: // AL1_CTRL
            case 0x20: // AL2_CTRL
            case 0x30: // AL3_CTRL
                return ctrl[ch] | (channelTriggered & (1 << ch) ? (1 << 24)/*BUSY*/ : 0);
        }
    }
    else
    {
        switch(addr)
        {
            case 0x400: // INTR
                return interrupts;
            case 0x404: // INTE0
                return interruptEnables[0];
            case 0x40C: // INTS0
                return interrupts & interruptEnables[0]; // TODO: force
            case 0x414: // INTE1
                return interruptEnables[1];
            case 0x41C: // INTS1
                return interrupts & interruptEnables[1]; // TODO: force
        }
        printf("DMA R %08X\n", addr);
    }
    return 0xBADADD55;
}

void DMA::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < 0x400)
    {
        int ch = addr / 0x40;
        assert(ch < numChannels);

        switch(addr & 0x3F)
        {
            case 0x00: // READ_ADDR
            case 0x14: // AL1_READ_ADDR
            case 0x28: // AL2_READ_ADDR
            case 0x3C: // AL3_READ_ADDR_TRIG
                updateReg(readAddr[ch], data, atomic);
                break;
            case 0x04: // WRITE_ADDR
            case 0x18: // AL1_WRITE_ADDR
            case 0x2C: // AL2_WRITE_ADDR_TRIG
            case 0x34: // AL3_WRITE_ADDR
                updateReg(writeAddr[ch], data, atomic);
                break;
            case 0x08: // TRANS_COUNT
            case 0x1C: // AL1_TRANS_COUNT_TRIG
            case 0x24: // AL2_TRANS_COUNT
            case 0x38: // AL3_TRANS_COUNT
                updateReg(transferCountReload[ch], data, atomic);
                break;
            case 0x0C: // CTRL_TRIG
            case 0x10: // AL1_CTRL
            case 0x20: // AL2_CTRL
            case 0x30: // AL3_CTRL
                updateReg(ctrl[ch], data & ~(1 << 24 | 1 << 31), atomic);
                break;
        }

        if((addr & 0xF) == 0xC && (ctrl[ch] & 1/*EN*/)) // *_TRIG
        {
            transferCount[ch] = transferCountReload[ch];
            channelTriggered |= 1 << ch;
        }
    }
    else
    {
        switch(addr)
        {
            case 0x404: // INTE0
                updateReg(interruptEnables[0], data, atomic);
                return;
            case 0x40C: // INTS0
            case 0x41C: // INTS1
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
            case 0x414: // INTE1
                updateReg(interruptEnables[1], data, atomic);
                return;
        }

        printf("DMA W %03X%s%08X\n", addr, op[atomic], data);
    }
}