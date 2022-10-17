#pragma once
#include <cassert>
#include <cstdint>

class MemoryBus
{
public:
    MemoryBus();

    void setBootROM(const uint8_t *rom);

    void reset();

    template<class T>
    T read(uint32_t addr, int &cycles, bool sequential) const;
    template<class T>
    void write(uint32_t addr, T data, int &cycles, bool sequential);

    const uint8_t *mapAddress(uint32_t addr) const;
    uint8_t *mapAddress(uint32_t addr);

    int getAccessCycles(uint32_t addr, int width, bool sequential) const;

    inline int iCycle(int i = 1)
    {
        return i;
    }

    int prefetchTiming16(int cycles, int bugCycles = 0)
    {
        return cycles;
    }

    // verify that pointer returns the same as a regular read to the address
    // without affecting prefetch (used for asserts)
    template<class T>
    bool verifyPointer(const T *ptr, uint32_t addr)
    {
        int tmp;
        bool ret = read<T>(addr, tmp, false) == *ptr;
        return ret;
    }

private:
    template<class T, size_t size>
    T doRead(const uint8_t (&mem)[size], uint32_t addr) const;
    template<class T, size_t size>
    void doWrite(uint8_t (&mem)[size], uint32_t addr, T data);

    template<class T>
    T doROMRead(uint32_t addr) const;

    template<class T>
    T doOpenRead(uint32_t addr) const;

    const uint8_t *bootROM = nullptr;

    uint8_t sram[264 * 1024];

    uint32_t dummy = 0xBADADD55;
};