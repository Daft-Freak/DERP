#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

enum MemoryRegion
{
    Region_ROM       = 0x00,
    Region_XIP       = 0x10,
    Region_XIP_SSI   = 0x18,
    Region_SRAM      = 0x20,
    Region_AHBPeriph = 0x50,
};

template uint8_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential) const;
template uint16_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential) const;
template uint32_t MemoryBus::read(uint32_t addr, int &cycles, bool sequential) const;
template void MemoryBus::write(uint32_t addr, uint8_t val, int &cycles, bool sequential);
template void MemoryBus::write(uint32_t addr, uint16_t val, int &cycles, bool sequential);
template void MemoryBus::write(uint32_t addr, uint32_t val, int &cycles, bool sequential);

MemoryBus::MemoryBus() {}

void MemoryBus::setBootROM(const uint8_t *rom)
{
    bootROM = rom;
}

void MemoryBus::reset()
{

}

template<class T>
T MemoryBus::read(uint32_t addr, int &cycles, bool sequential) const
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
    };

    switch(addr >> 24)
    {
        case Region_ROM:
            accessCycles(1);
            return doROMRead<T>(addr);

        case Region_XIP_SSI:
            accessCycles(1);
            return doXIPSSIRead<T>(addr);

        case Region_SRAM:
            accessCycles(1);
            return doSRAMRead<T>(addr);

        case Region_AHBPeriph:
            accessCycles(1);
            return doAHBPeriphRead<T>(addr);

    }

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::write(uint32_t addr, T data, int &cycles, bool sequential)
{
    auto accessCycles = [&cycles, this](int c)
    {
        cycles += c;
    };

    switch(addr >> 24)
    {
        case Region_ROM:
            accessCycles(1);
            return;

        case Region_XIP_SSI:
            accessCycles(1);
            doXIPSSIWrite<T>(addr, data);
            return;

        case Region_SRAM:
            accessCycles(1);
            doSRAMWrite<T>(addr, data);
            return;

        case Region_AHBPeriph:
            accessCycles(1);
            doAHBPeriphWrite<T>(addr, data);
            return;

    }
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case Region_ROM:
            return bootROM ? bootROM + (addr & 0x3FFF) : nullptr;

    }

    return reinterpret_cast<const uint8_t *>(&dummy);
}

uint8_t *MemoryBus::mapAddress(uint32_t addr)
{
    switch(addr >> 24)
    {
        case Region_SRAM:
        {
            if(addr < 0x20040000)
            {
                // striped SRAM0-3
            }
            else
            {
                // SRAM4-5 (or OOB)
                if(addr < 0x20042000)
                    return sram + (addr & 0xFFFFF);
            }

            break;
        }

        case Region_AHBPeriph:
        {
            if(addr >= 0x50100000 && addr < 0x50101000)
                return usbDPRAM + (addr & 0xFFF);
        }
    }

    return nullptr;
}

int MemoryBus::getAccessCycles(uint32_t addr, int width, bool sequential) const
{
    return 1;
}

template<class T, size_t size>
T MemoryBus::doRead(const uint8_t (&mem)[size], uint32_t addr) const
{
    // use size of type for alignment
    return *reinterpret_cast<const T *>(mem + (addr & (size - sizeof(T))));
}

template<class T, size_t size>
void MemoryBus::doWrite(uint8_t (&mem)[size], uint32_t addr, T data)
{
    // use size of type for alignment
    *reinterpret_cast< T *>(mem + (addr & (size - sizeof(T)))) = data;
}

template<class T>
T MemoryBus::doROMRead(uint32_t addr) const
{
    if(!bootROM)
        return 0; 

    const size_t size = 0x4000;
    return *reinterpret_cast<const T *>(bootROM + (addr & (size - 1)));
}

template<class T>
T MemoryBus::doXIPSSIRead(uint32_t addr) const
{
    printf("XIP SSI R %08X\n", addr);
    return 0;
}

template<class T>
void MemoryBus::doXIPSSIWrite(uint32_t addr, T data)
{
    printf("XIP SSI W %08X -> %08X\n", data, addr);
}

template<class T>
T MemoryBus::doSRAMRead(uint32_t addr) const
{
    if(addr < 0x20040000)
    {
        // striped SRAM0-3
    }
    else if (addr < 0x20042000) // SRAM4-5
        return *reinterpret_cast<const T *>(sram + (addr & 0xFFFFF));

    printf("SRAM R %08X\n", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doSRAMWrite(uint32_t addr, T data)
{
    if(addr < 0x20040000)
    {
        // striped SRAM0-3
    }
    else if (addr < 0x20042000) // SRAM4-5
    {
        *reinterpret_cast<T *>(sram + (addr & 0xFFFFF)) = data;
    }

    printf("SRAM W %08X\n", addr);
}

template<class T>
T MemoryBus::doAHBPeriphRead(uint32_t addr) const
{
    // USB DPRAM
    if(addr >= 0x50100000 && addr < 0x50101000)
        return doRead<T>(usbDPRAM, addr);

    printf("AHBP R %08X\n", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doAHBPeriphWrite(uint32_t addr, T data)
{
    // USB DPRAM
    if(addr >= 0x50100000 && addr < 0x50101000)
    {
        doWrite<T>(usbDPRAM, addr, data);
        return;
    }

    printf("AHBP W %08X\n", addr);
}

template<class T>
T MemoryBus::doOpenRead(uint32_t addr) const
{
    return static_cast<T>(0xBADADD55); // TODO
}
