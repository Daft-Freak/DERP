#pragma once
#include <cstddef>
#include <cstdint>
#include <queue>
#include <variant>

#include "Clocks.h"
#include "DMA.h"
#include "GPIO.h"
#include "Timer.h"
#include "Watchdog.h"

class ARMv6MCore;

bool updateReg(uint32_t &curVal, uint32_t newVal, int atomic);

using BusMasterPtr = std::variant<ARMv6MCore *, DMA *>;

class MemoryBus
{
public:
    MemoryBus();

    void setBootROM(const uint8_t *rom);

    void setCPU(ARMv6MCore *cpu);

    void reset();

    template<class T>
    T read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential);
    template<class T>
    void write(BusMasterPtr master, uint32_t addr, T data, int &cycles, bool sequential);

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
    bool verifyPointer(ARMv6MCore &cpu, const T *ptr, uint32_t addr)
    {
        int tmp;
        bool ret = read<T>(&cpu, addr, tmp, false) == *ptr;
        return ret;
    }

    void peripheralUpdate(uint64_t target);
    void peripheralUpdate(uint64_t target, uint32_t irqMask);
    uint64_t getNextInterruptTime(uint64_t target);

    void setPendingIRQ(int n);

    Clocks &getClocks() {return clocks;}
    GPIO &getGPIO() {return gpio;}
    Watchdog &getWatchdog() {return watchdog;}

private:
    template<class T, size_t size>
    T doRead(const uint8_t (&mem)[size], uint32_t addr) const;
    template<class T, size_t size>
    void doWrite(uint8_t (&mem)[size], uint32_t addr, T data);

    template<class T>
    T doROMRead(uint32_t addr) const;

    template<class T>
    T doXIPSSIRead(uint32_t addr);
    template<class T>
    void doXIPSSIWrite(uint32_t addr, T data);

    template<class T>
    T doSRAMRead(uint32_t addr) const;
    template<class T>
    void doSRAMWrite(uint32_t addr, T data);

    template<class T>
    T doAPBPeriphRead(ClockTarget &masterClock, uint32_t addr);
    template<class T>
    void doAPBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data);

    template<class T>
    T doAHBPeriphRead(ClockTarget &masterClock, uint32_t addr);
    template<class T>
    void doAHBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data);

    template<class T>
    T doIOPORTRead(uint32_t addr);
    template<class T>
    void doIOPORTWrite(uint32_t addr, T data);

    template<class T>
    T doCPUInternalRead(ARMv6MCore &cpu, uint32_t addr) const;
    template<class T>
    void doCPUInternalWrite(ARMv6MCore &cpu, uint32_t addr, T data);

    template<class T>
    T doOpenRead(uint32_t addr) const;

    const uint8_t *bootROM = nullptr;

    uint8_t qspiFlash[16 * 1024 * 1024]; // max

    uint8_t sram[264 * 1024]; // first 256k pre-striped

    uint8_t usbDPRAM[4 * 1024];

    uint32_t dummy = 0xBADADD55;

    ARMv6MCore *cpu = nullptr; // TODO: cpus

    Clocks clocks;

    GPIO gpio;

    Watchdog watchdog;

    Timer timer; // depends on watchdog

    DMA dma;

    // temp peripherals stuff
    uint32_t ioQSPICtrl[6]{0};

    uint32_t pwmCSR[8];
    uint32_t pwmDIV[8];
    uint32_t pwmCTR[8];
    uint32_t pwmCC[8];
    uint32_t pwmTOP[8];

    std::queue<uint32_t> ssiRx;

    uint8_t flashCmd;
    uint32_t flashAddr;
    int flashCmdOff = 0;

    // TODO: there are two of these
    uint32_t dividend, divisor;
    uint32_t divQuot, divRem;
    bool dividerSigned, dividerDirty = false;

    uint32_t spinlocks = 0;
};