#pragma once
#include <cassert>
#include <cstdint>
#include <queue>

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
    T doXIPSSIRead(uint32_t addr) const;
    template<class T>
    void doXIPSSIWrite(uint32_t addr, T data);

    template<class T>
    T doSRAMRead(uint32_t addr) const;
    template<class T>
    void doSRAMWrite(uint32_t addr, T data);

    template<class T>
    T doAPBPeriphRead(uint32_t addr) const;
    template<class T>
    void doAPBPeriphWrite(uint32_t addr, T data);

    template<class T>
    T doAHBPeriphRead(uint32_t addr) const;
    template<class T>
    void doAHBPeriphWrite(uint32_t addr, T data);

    template<class T>
    T doIOPORTRead(uint32_t addr) const;
    template<class T>
    void doIOPORTWrite(uint32_t addr, T data);

    template<class T>
    T doCPUInternalRead(uint32_t addr) const;
    template<class T>
    void doCPUInternalWrite(uint32_t addr, T data);

    template<class T>
    T doOpenRead(uint32_t addr) const;

    const uint8_t *bootROM = nullptr;

    uint8_t qspiFlash[16 * 1024 * 1024]; // max

    uint8_t sram[264 * 1024];

    uint8_t usbDPRAM[4 * 1024];

    uint32_t dummy = 0xBADADD55;

    // temp peripherals stuff
    uint32_t ioQSPICtrl[6]{0};

    mutable std::queue<uint32_t> ssiRx; //!

    uint8_t flashCmd;
    uint32_t flashAddr;
    int flashCmdOff = 0;
};