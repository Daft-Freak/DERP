#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <variant>

#include "Clocks.h"
#include "DMA.h"
#include "GPIO.h"
#include "FIFO.h"
#include "I2C.h"
#include "PIO.h"
#include "PWM.h"
#include "Timer.h"
#include "UART.h"
#include "USB.h"
#include "Watchdog.h"

class ARMv6MCore;

bool updateReg(uint32_t &curVal, uint32_t newVal, int atomic);

using BusMasterPtr = std::variant<ARMv6MCore *, DMA *>;

class MemoryBus
{
public:
    using InterruptUpdateCallback = std::function<void(uint64_t, uint32_t)>;
    using GetNextInterruptTimeCallback = std::function<uint64_t(uint64_t)>;

    MemoryBus();

    void setBootROM(const uint8_t *rom);

    void setCPUs(ARMv6MCore *cpus);

    void reset();

    template<class T>
    T read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential);
    template<class T>
    void write(BusMasterPtr master, uint32_t addr, T data, int &cycles, bool sequential);

    const uint8_t *mapAddress(uint32_t addr) const;
    uint8_t *mapAddress(uint32_t addr);

    int getAccessCycles(uint32_t addr, int width, bool sequential) const;

    void updatePC(uint32_t pc);

    inline int iCycle(int i = 1)
    {
        return i;
    }

    int fetchTiming(uint32_t addr, int cycles)
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
    void peripheralUpdate(uint64_t target, uint32_t irqMask, ARMv6MCore *core);

    void gpioUpdate(uint64_t target);

    uint64_t getNextInterruptTime() const {return nextInterruptTime;}
    void calcNextInterruptTime();

    void setInterruptUpdateCallback(InterruptUpdateCallback cb);
    void setGetNextInterruptTimeCallback(GetNextInterruptTimeCallback cb);

    void setPendingIRQ(int n);

    void sendEvent(ARMv6MCore *coreFrom);

    Clocks &getClocks() {return clocks;}
    GPIO &getGPIO() {return gpio;}
    PWM &getPWM() {return pwm;}
    Watchdog &getWatchdog() {return watchdog;}

    DMA &getDMA() {return dma;}
    USB &getUSB() {return usb;}
    PIO &getPIO(int i) {return pio[i];}

private:
    template<class T, size_t size>
    T doRead(const uint8_t (&mem)[size], uint32_t addr) const;
    template<class T, size_t size>
    void doWrite(uint8_t (&mem)[size], uint32_t addr, T data);

    template<class T>
    T doROMRead(uint32_t addr) const;

    template<class T>
    T doXIPCtrlRead(uint32_t addr);
    template<class T>
    void doXIPCtrlWrite(uint32_t addr, T data);

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
    T doIOPORTRead(ClockTarget &masterClock, int core, uint32_t addr);
    template<class T>
    void doIOPORTWrite(ClockTarget &masterClock, int core, uint32_t addr, T data);

    template<class T>
    T doCPUInternalRead(ARMv6MCore &cpu, uint32_t addr) const;
    template<class T>
    void doCPUInternalWrite(ARMv6MCore &cpu, uint32_t addr, T data);

    template<class T>
    T doOpenRead(uint32_t addr) const;

    bool pcInCachedXIP = false;

    const uint8_t *bootROM = nullptr;

    uint8_t qspiFlash[16 * 1024 * 1024]; // max
    uint8_t xipCache[16 * 1024];

    uint8_t sram[264 * 1024]; // first 256k pre-striped

    uint32_t dummy = 0xBADADD55;

    ARMv6MCore *cpuCores = nullptr;

    Clocks clocks;

    GPIO gpio;

    UART uart[2];

    I2C i2c[2];

    PWM pwm;

    Watchdog watchdog;

    Timer timer; // depends on watchdog

    DMA dma;

    USB usb;

    PIO pio[2];

    uint64_t nextInterruptTime;

    InterruptUpdateCallback interruptUpdateCallback;
    GetNextInterruptTimeCallback getNextInterruptTimeCallback;

    // temp peripherals stuff
    uint32_t ioQSPICtrl[6]{0};

    uint32_t rtcCtrl = 0;

    uint32_t xipCtrlCtrl = 0;

    std::queue<uint32_t> ssiRx;

    uint8_t flashCmd;
    uint32_t flashAddr;
    int flashCmdOff = 0;

    uint32_t psmOff = 0;

    // inter-core FIFO
    FIFO<uint32_t, 8> coreFIFO[2];

    // per-core divider
    uint32_t dividend[2], divisor[2];
    uint32_t divQuot[2], divRem[2];
    bool dividerSigned[2], dividerDirty[2];

    uint32_t spinlocks = 0;
};
