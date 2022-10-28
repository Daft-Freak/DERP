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

    channelTriggered = 0;
}

void DMA::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);

    clock.addCycles(passed);
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
                return ctrl[ch];
        }
    }
    else
        printf("DMA R %08X\n", addr);
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
                updateReg(ctrl[ch], data, atomic);
                break;
        }

        if((addr & 0xF) == 0xC && (ctrl[ch] & 1/*EN*/)) // *_TRIG
        {
            printf("DMA trig %i (%08X -> %08X x %i)\n", ch, readAddr[ch], writeAddr[ch], transferCountReload[ch]);
            transferCount[ch] = transferCountReload[ch];
            channelTriggered |= 1 << ch;
        }
    }
    else
        printf("DMA W %03X%s%08X\n", addr, op[atomic], data);
}