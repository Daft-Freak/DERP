#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

enum MemoryRegion
{
    Region_ROM      = 0x0,
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

    switch(addr >> 28)
    {
        case Region_ROM:
            accessCycles(1);
            return doROMRead<T>(addr);
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
T MemoryBus::doOpenRead(uint32_t addr) const
{
    return static_cast<T>(0xBADADD55); // TODO
}
