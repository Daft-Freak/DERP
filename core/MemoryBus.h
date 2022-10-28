#pragma once
#include <cassert>
#include <cstdint>
#include <queue>

#include "Clocks.h"
#include "GPIO.h"

class ARMv6MCore;

class MemoryBus;

class Watchdog final
{
public:
    Watchdog();

    void reset();

    void update(uint64_t target);

    uint32_t getTicks();

    uint64_t getTickTarget(uint32_t numTicks);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    uint32_t ctrl;
    uint32_t scratch[8];
    uint32_t tick;

    ClockTarget clock;

    int timer;
    int tickCounter;
    uint32_t ticks;
};

class Timer final
{
public:
    Timer(MemoryBus &mem);

    void reset();

    void update(uint64_t target);

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

private:
    MemoryBus &mem;

    uint64_t time;
    uint32_t latchedHighTime, writeLowTime;

    uint32_t alarms[4];
    uint32_t armed;

    uint32_t interrupts;
    uint32_t interruptEnables;

    uint32_t lastTicks;
};

class MemoryBus
{
public:
    MemoryBus();

    void setBootROM(const uint8_t *rom);

    void setCPU(ARMv6MCore *cpu);

    void reset();

    template<class T>
    T read(ARMv6MCore &cpu, uint32_t addr, int &cycles, bool sequential);
    template<class T>
    void write(ARMv6MCore &cpu, uint32_t addr, T data, int &cycles, bool sequential);

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
        bool ret = read<T>(cpu, addr, tmp, false) == *ptr;
        return ret;
    }

    void peripheralUpdate(uint64_t target);
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
    T doAPBPeriphRead(ARMv6MCore &cpu, uint32_t addr);
    template<class T>
    void doAPBPeriphWrite(ARMv6MCore &cpu, uint32_t addr, T data);

    template<class T>
    T doAHBPeriphRead(uint32_t addr) const;
    template<class T>
    void doAHBPeriphWrite(uint32_t addr, T data);

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

    uint8_t sram[264 * 1024];

    uint8_t usbDPRAM[4 * 1024];

    uint32_t dummy = 0xBADADD55;

    ARMv6MCore *cpu = nullptr; // TODO: cpus

    Clocks clocks;

    GPIO gpio;

    Watchdog watchdog;

    Timer timer; // depends on watchdog

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