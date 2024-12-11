#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "hardware/regs/addressmap.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/io_qspi.h" // TODO: move
#include "hardware/regs/pio.h" // TODO
#include "hardware/regs/psm.h" // TODO
#include "hardware/regs/resets.h" // TODO
#include "hardware/regs/rtc.h" // TODO
#include "hardware/regs/spi.h" // TODO
#include "hardware/regs/ssi.h" // TODO: move
#include "hardware/regs/sio.h" // TODO
#include "hardware/regs/sysinfo.h" // TODO
#include "hardware/structs/clocks.h" // clock_index
#include "hardware/structs/usb.h" // USB_DPRAM_SIZE

#include "MemoryBus.h"

#include "ARMv6MCore.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::MemoryBus;

bool updateReg(uint32_t &curVal, uint32_t newVal, int atomic)
{
    auto oldVal = curVal;

    if(atomic == 0)
        curVal = newVal;
    else if(atomic == 1)
        curVal ^= newVal;
    else if(atomic == 2)
        curVal |= newVal;
    else
        curVal &= ~newVal;

    return curVal != oldVal;
}

// this is the high 8 bits of the address
enum MemoryRegion
{
    Region_ROM           = ROM_BASE >> 24,
    Region_XIP           = XIP_BASE >> 24,
    Region_XIP_NoAlloc   = XIP_NOALLOC_BASE >> 24,
    Region_XIP_NoCache   = XIP_NOCACHE_BASE >> 24,
    Region_XIP_NoCNoA    = XIP_NOCACHE_NOALLOC_BASE >> 24,
    Region_XIP_Ctrl      = XIP_CTRL_BASE >> 24,
    Region_XIP_CacheSRAM = XIP_SRAM_BASE >> 24,
    Region_XIP_SSI       = XIP_SSI_BASE >> 24,
    Region_SRAM          = SRAM_BASE >> 24,
    Region_APBPeriph     = SYSINFO_BASE >> 24, // first periph
    Region_AHBPeriph     = DMA_BASE >> 24, // first periph
    Region_IOPORT        = SIO_BASE >> 24,
    Region_CPUInternal   = PPB_BASE >> 24,
};

// peripherals
#define PERIPH(x) ((x##_BASE >> 14) & 0x1F)

enum class APBPeripheral
{
    SysInfo = PERIPH(SYSINFO),
    SysCfg = PERIPH(SYSCFG),
    Clocks = PERIPH(CLOCKS),
    Resets = PERIPH(RESETS),
    PSM = PERIPH(PSM),
    IO_Bank0 = PERIPH(IO_BANK0),
    IO_QSPI = PERIPH(IO_QSPI),
    Pads_Bank0 = PERIPH(PADS_BANK0),
    Pads_QSPI = PERIPH(PADS_QSPI),
    XOsc = PERIPH(XOSC),
    PLL_Sys = PERIPH(PLL_SYS),
    PLL_USB = PERIPH(PLL_USB),
    BusCtrl = PERIPH(BUSCTRL),
    UART0 = PERIPH(UART0),
    UART1 = PERIPH(UART1),
    SPI0 = PERIPH(SPI0),
    SPI1 = PERIPH(SPI1),
    I2C0 = PERIPH(I2C0),
    I2C1 = PERIPH(I2C1),
    ADC = PERIPH(ADC),
    PWM = PERIPH(PWM),
    Timer = PERIPH(TIMER),
    Watchdog = PERIPH(WATCHDOG),
    RTC = PERIPH(RTC),
    ROsc = PERIPH(ROSC),
    VRegAndChipReset = PERIPH(VREG_AND_CHIP_RESET),
    TBMan = PERIPH(TBMAN),
};

#undef PERIPH
#define PERIPH(x) ((x##_BASE >> 20) & 0xF)

enum class AHBPeripheral
{
    DMA = PERIPH(DMA),
    USB = PERIPH(USBCTRL),
    PIO0 = PERIPH(PIO0),
    PIO1 = PERIPH(PIO1),
    XIPAux = PERIPH(XIP_AUX),
};

#undef PERIPH

template uint8_t MemoryBus::read(ClockedDevice *master, uint32_t addr, int &cycles);
template uint16_t MemoryBus::read(ClockedDevice *master, uint32_t addr, int &cycles);
template uint32_t MemoryBus::read(ClockedDevice *master, uint32_t addr, int &cycles);
template void MemoryBus::write(ClockedDevice *master, uint32_t addr, uint8_t val, int &cycles);
template void MemoryBus::write(ClockedDevice *master, uint32_t addr, uint16_t val, int &cycles);
template void MemoryBus::write(ClockedDevice *master, uint32_t addr, uint32_t val, int &cycles);

static inline uint32_t getStripedSRAMAddr(uint32_t addr)
{
    int bank = (addr >> 2) & 3;
    int word = (addr >> 4) & 0xFFFF; // a few too many bits

    return bank * 64 * 1024 + word * 4 + (addr & 3);
}

MemoryBus::MemoryBus() : gpio(*this), uart{{*this, 0}, {*this, 1}}, i2c{{*this, 0}, {*this, 1}}, pwm(*this), watchdog(*this), timer(*this), dma(*this), usb(*this), pio{{*this, 0}, {*this, 1}}
{
    clocks.addClockTarget(clk_ref, watchdog.getClock());

    clocks.addClockTarget(clk_sys, gpio.getClock());
    clocks.addClockTarget(clk_sys, pwm.getClock());
    
    clocks.addClockTarget(clk_sys, dma.getClock());
    clocks.addClockTarget(clk_usb, usb.getClock());
    clocks.addClockTarget(clk_sys, pio[0].getClock());
    clocks.addClockTarget(clk_sys, pio[1].getClock());
}

void MemoryBus::setBootROM(const uint8_t *rom)
{
    bootROM = rom;
}

void MemoryBus::setCPUs(ARMv6MCore *cpus)
{
    this->cpuCores = cpus;
}

void MemoryBus::reset()
{
    cpuCores[0].reset();
    cpuCores[1].reset();

    clocks.reset();
    gpio.reset();
    uart[0].reset();
    uart[1].reset();
    i2c[0].reset();
    i2c[1].reset();
    pwm.reset();
    timer.reset();
    watchdog.reset();

    dma.reset();
    usb.reset();
    pio[0].reset();
    pio[1].reset();

    nextInterruptTime = 0;

    for(auto &dirty : dividerDirty)
        dirty = false;
}

template<class T>
T MemoryBus::read(ClockedDevice *master, uint32_t addr, int &cycles)
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

        case Region_XIP:
        case Region_XIP_NoAlloc: // TODO: implement the cache
        case Region_XIP_NoCache:
        case Region_XIP_NoCNoA:
            accessCycles(1);
            return doRead<T>(qspiFlash, addr);

        case Region_XIP_Ctrl:
            accessCycles(1);
            return doXIPCtrlRead<T>(addr);

        case Region_XIP_CacheSRAM:
            if(!(xipCtrlCtrl & 1)) // cache disabled
            {
                accessCycles(1);
                return doRead<T>(xipCache, addr); // this one mirrors
            }
            break;

        case Region_XIP_SSI:
            accessCycles(1);
            return doXIPSSIRead<T>(addr);

        case Region_SRAM:
            accessCycles(1);
            return doSRAMRead<T>(addr);

        case Region_APBPeriph:
            accessCycles(3);
            return doAPBPeriphRead<T>(master->getClock(), addr);

        case Region_AHBPeriph:
            accessCycles(1);
            return doAHBPeriphRead<T>(master->getClock(), addr);

        case Region_IOPORT:
        {
            assert(master->getDeviceFlags() & ClockedDevice_CPU);
            int core = static_cast<ARMv6MCore *>(master) - cpuCores;
            return doIOPORTRead<T>(master->getClock(), core, addr);
        }

        case Region_CPUInternal:
            accessCycles(1);
            assert(master->getDeviceFlags() & ClockedDevice_CPU);
            return doCPUInternalRead<T>(*static_cast<ARMv6MCore *>(master), addr);
    }

    if(master->getDeviceFlags() & ClockedDevice_CPU)
        static_cast<ARMv6MCore *>(master)->fault("Memory read");

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::write(ClockedDevice *master, uint32_t addr, T data, int &cycles)
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

        case Region_XIP_Ctrl:
            accessCycles(1);
            doXIPCtrlWrite<T>(addr, data);
            return;

        case Region_XIP_CacheSRAM:
            if(!(xipCtrlCtrl & 1)) // cache disabled
            {
                accessCycles(1);
                doWrite<T>(xipCache, addr, data); // this one mirrors
                return;
            }
            break;

        case Region_XIP_SSI:
            accessCycles(1);
            doXIPSSIWrite<T>(addr, data);
            return;

        case Region_SRAM:
            accessCycles(1);
            doSRAMWrite<T>(addr, data);
            return;

        case Region_APBPeriph:
            accessCycles(4);
            doAPBPeriphWrite<T>(master->getClock(), addr, data);
            return;

        case Region_AHBPeriph:
            accessCycles(1);
            doAHBPeriphWrite<T>(master->getClock(), addr, data);
            return;

        case Region_IOPORT:
        {
            assert(master->getDeviceFlags() & ClockedDevice_CPU);
            int core = static_cast<ARMv6MCore *>(master) - cpuCores;
            doIOPORTWrite<T>(master->getClock(), core, addr, data);
            return;
        }   

        case Region_CPUInternal:
            accessCycles(1);
            assert(master->getDeviceFlags() & ClockedDevice_CPU);
            doCPUInternalWrite<T>(*static_cast<ARMv6MCore *>(master), addr, data);
            return;
    }

    if(master->getDeviceFlags() & ClockedDevice_CPU)
        static_cast<ARMv6MCore *>(master)->fault("Memory write");
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case Region_ROM:
            return bootROM ? bootROM + (addr & 0x3FFF) : nullptr;

        case Region_XIP:
            return qspiFlash + (addr & 0xFFFFFF);

        case Region_XIP_CacheSRAM:
            return xipCache + (addr & 0x3FFF);

        case Region_SRAM:
        {
            // SRAM0-3 is stored striped so that we can do this
            // + SRAM4-5
            if(addr < SRAM_END)
                return sram + (addr & 0xFFFFF);

            break;
        }

    }

    return reinterpret_cast<const uint8_t *>(&dummy);
}

uint8_t *MemoryBus::mapAddress(uint32_t addr)
{
    switch(addr >> 24)
    {
        case Region_XIP:
            return qspiFlash + (addr & 0xFFFFFF);

        case Region_XIP_CacheSRAM:
            return xipCache + (addr & 0x3FFF);

        case Region_SRAM:
        {
            // SRAM0-3 (striped) + SRAM4-5
            if(addr < SRAM_END)
                return sram + (addr & 0xFFFFF);

            break;
        }

        case Region_AHBPeriph:
        {
            if(addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
                return usb.getRAM() + (addr & (USB_DPRAM_SIZE - 1));
        }
    }

    return nullptr;
}

int MemoryBus::getAccessCycles(uint32_t addr, int width) const
{
    return 1;
}

void MemoryBus::updatePC(uint32_t pc)
{
    // TODO: could be non-alloc cached
    pcInCachedXIP = (pc >> 24) == Region_XIP;
}

void MemoryBus::peripheralUpdate(uint64_t target)
{
    ClockedDevice *devices[]
    {
        &pwm,
        &timer,
        &dma,
        &usb,
        &pio[0],
        &pio[1],
    };
    syncDevices(target, devices, std::size(devices));

    gpio.update(target);
}

void MemoryBus::gpioUpdate(uint64_t target)
{
    // update anything that might affect outputs before GPIO
    ClockedDevice *devices[]
    {
        &pwm,
        // technically affects IO, but not implemented yet
        // would also need to sync DMA most of the time
        //&pio[0],
        //&pio[1],
    };
    syncDevices(target, devices, std::size(devices));

    gpio.update(target);
}

void MemoryBus::peripheralUpdate(uint64_t target, uint32_t irqMask, ARMv6MCore *core)
{
    if(interruptUpdateCallback)
        interruptUpdateCallback(target, irqMask);

    // sync core 1
    if(core == cpuCores) // core 0
        cpuCores[1].update(target);

    // only update things that might cause interrupts
    ClockedDevice *devices[6];
    int numDevices = 0;

    const auto timerIRQs = 1 << TIMER_IRQ_0 | 1 << TIMER_IRQ_1 | 1 << TIMER_IRQ_2 | 1 << TIMER_IRQ_3;
    if((irqMask & timerIRQs) && timer.needUpdateForInterrupts())
        devices[numDevices++] = &timer;

    if((irqMask & (1 << PWM_IRQ_WRAP)) && pwm.needUpdateForInterrupts())
        devices[numDevices++] = &pwm;

    if((irqMask & (1 << DMA_IRQ_0 | 1 << DMA_IRQ_1)) && dma.needUpdateForInterrupts())
        devices[numDevices++] = &dma;

    if((irqMask & (1 << USBCTRL_IRQ)) && usb.needUpdateForInterrupts())
        devices[numDevices++] = &usb;

    if(irqMask & (1 << PIO0_IRQ_0 | 1 << PIO0_IRQ_1))
        devices[numDevices++] = &pio[0];

    if(irqMask & (1 << PIO1_IRQ_0 | 1 << PIO1_IRQ_1))
        devices[numDevices++] = &pio[1];

    syncDevices(target, devices, numDevices);
}

void MemoryBus::calcNextInterruptTime()
{
    // TODO: check cpu enabled mask
    nextInterruptTime = ~0ull;

    if(getNextInterruptTimeCallback)
        nextInterruptTime = getNextInterruptTimeCallback(nextInterruptTime);

    nextInterruptTime = pwm.getNextInterruptTime(nextInterruptTime);
    nextInterruptTime = timer.getNextInterruptTime(nextInterruptTime);
    nextInterruptTime = dma.getNextInterruptTime(nextInterruptTime);
    nextInterruptTime = usb.getNextInterruptTime(nextInterruptTime);
    nextInterruptTime = pio[0].getNextInterruptTime(nextInterruptTime);
    nextInterruptTime = pio[1].getNextInterruptTime(nextInterruptTime);
}

void MemoryBus::setInterruptUpdateCallback(InterruptUpdateCallback cb)
{
    interruptUpdateCallback = cb;
}

void MemoryBus::setGetNextInterruptTimeCallback(GetNextInterruptTimeCallback cb)
{
    getNextInterruptTimeCallback = cb;
}

void MemoryBus::setPendingIRQ(int n)
{
    // TODO: and the other one
    cpuCores[0].setPendingIRQ(n);
    cpuCores[1].setPendingIRQ(n);

    calcNextInterruptTime();
}

void MemoryBus::sendEvent(ARMv6MCore *coreFrom)
{
    int coreIndex = coreFrom - cpuCores;
    cpuCores[1 - coreIndex].setEvent();
}

void MemoryBus::updateInterpolatorResult(interp_hw_t &interp)
{
    // lane 0/1
    for(int lane = 0; lane < 2; lane++)
    {
        auto ctrl = interp.ctrl[lane];

        auto base = interp.base[lane];
        auto accum = (ctrl & SIO_INTERP0_CTRL_LANE0_CROSS_INPUT_BITS) ? interp.accum[lane ^ 1] : interp.accum[lane];

        uint32_t shifted = accum;

        // shift
        shifted >>= (ctrl & SIO_INTERP0_CTRL_LANE0_SHIFT_BITS);

        // mask
        auto lsb = (ctrl & SIO_INTERP0_CTRL_LANE0_MASK_LSB_BITS) >> SIO_INTERP0_CTRL_LANE0_MASK_LSB_LSB;
        auto msb = (ctrl & SIO_INTERP0_CTRL_LANE0_MASK_MSB_BITS) >> SIO_INTERP0_CTRL_LANE0_MASK_MSB_LSB;
        uint32_t mask = (msb == 31 ? 0xFFFFFFFF : (1 << (msb - lsb + 1)) - 1) << lsb;
        shifted &= mask;

        // sign extend
        if((ctrl & SIO_INTERP0_CTRL_LANE0_SIGNED_BITS) && (shifted & (1 << msb)))
            shifted |= 0xFFFFFFFF << msb;
        
        uint32_t result = (ctrl & SIO_INTERP0_CTRL_LANE0_ADD_RAW_BITS) ? accum : shifted;

        interp.peek[lane] = result + base;

        // "raw" value is always shifted
        interp.add_raw[lane] = shifted;
    }

    interp.peek[2] = interp.base[2] + interp.add_raw[0] + interp.add_raw[1];
}

void MemoryBus::popInterpolator(interp_hw_t &interp)
{
    for(int lane = 0; lane < 2; lane++)
    {
        int outAccumIndex = (interp.ctrl[lane] & SIO_INTERP0_CTRL_LANE0_CROSS_RESULT_BITS) ? lane ^ 1 : lane;
        interp.accum[outAccumIndex] = interp.peek[lane];
    }

    // calculate next values
    updateInterpolatorResult(interp);
}

bool MemoryBus::syncDevices(uint64_t target, ClockedDevice **devices, int numDevices)
{
    // sort devices (most behind first)
    std::sort(devices, devices + numDevices, [](auto &a, auto &b){return a->getClock().getTime() < b->getClock().getTime();});

    int devIndex = 0;

    bool didUpdate = false;

    while(numDevices)
    {
        auto dev = devices[devIndex];
        auto time = dev->getClock().getTime();
        auto nextTarget = devIndex == numDevices - 1 ? target : devices[devIndex + 1]->getClock().getTime();

        dev->update(nextTarget);

        if(dev->getClock().getTime() == time)
        {
            // no progress
            if(devIndex == numDevices - 1)
            {
                // if this was the last (most up-to-date), we've reached the target, so drop this dev
                devIndex--;
                numDevices--;
            }
            else if(devIndex > 0 && didUpdate)
                devIndex--; // walk back up
            else
                devIndex++;

            didUpdate = false;
        }
        else
            didUpdate = true;
    }

    return true;
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

static const char *ctrlRegNames[]
{
    "CTRL",
    "FLUSH",
    "STAT",
    "CTR_HIT",
    "CTR_ACC",
    "STREAM_ADDR",
    "STREAM_CTR",
    "STREAM_FIFO",
};

template<class T>
T MemoryBus::doXIPCtrlRead(uint32_t addr)
{
    if(addr == 0)
        return xipCtrlCtrl;

    logf(LogLevel::NotImplemented, logComponent, "XIP ctrl R %s (%08X)", ctrlRegNames[(addr & 0xFF) / 4], addr);
    return 0;
}

template<class T>
void MemoryBus::doXIPCtrlWrite(uint32_t addr, T data)
{
    if(addr == 0)
        xipCtrlCtrl = data;

    logf(LogLevel::NotImplemented, logComponent, "XIP ctrl W %s (%08X) = %08X", ctrlRegNames[(addr & 0xFF) / 4], addr, data);
}

static const char *ssiRegNames[]
{
    "CTRLR0",
    "CTRLR1",
    "SSIENR",
    "MWCR",
    "SER",
    "BAUDR",
    "TXFTLR",
    "RXFTLR",
    "TXFLR",
    "RXFLR",
    "SR",
    "IMR",
    "ISR",
    "RISR",
    "TXOICR",
    "RXOICR",
    "RXUICR",
    "MSTICR",
    "ICR",
    "DMACR",
    "DMATDLR",
    "DMARDLR",
    "IDR",
    "SSI_VERSION_ID",
    "DR0",
    "DR1",
    "DR2",
    "DR3",
    "DR4",
    "DR5",
    "DR6",
    "DR7",
    "DR8",
    "DR9",
    "DR10",
    "DR11",
    "DR12",
    "DR13",
    "DR14",
    "DR15",
    "DR16",
    "DR17",
    "DR18",
    "DR19",
    "DR20",
    "DR21",
    "DR22",
    "DR23",
    "DR24",
    "DR25",
    "DR26",
    "DR27",
    "DR28",
    "DR29",
    "DR30",
    "DR31",
    "DR32",
    "DR33",
    "DR34",
    "DR35",
    "RX_SAMPLE_DLY",
    "SPI_CTRLR0",
    "TXD_DRIVE_EDGE",
};

template<class T>
T MemoryBus::doXIPSSIRead(uint32_t addr)
{
    switch(addr & 0xFF)
    {
        case SSI_TXFLR_OFFSET:
            return 0;
        case SSI_RXFLR_OFFSET:
            return static_cast<T>(ssiRx.size());

        case SSI_SR_OFFSET: // SR
            return SSI_SR_TFNF_BITS | SSI_SR_TFE_BITS | (ssiRx.empty() ? 0 : SSI_SR_RFNE_BITS);

        case SSI_DR0_OFFSET:
        {
            uint32_t v = 0;
            if(!ssiRx.empty())
            {
                v = ssiRx.front();
                ssiRx.pop();
            }
            return v;
        }
    }
    logf(LogLevel::NotImplemented, logComponent, "XIP SSI R %s (%08X)", ssiRegNames[(addr & 0xFF) / 4], addr);
    return 0;
}

template<class T>
void MemoryBus::doXIPSSIWrite(uint32_t addr, T data)
{
    switch(addr & 0xFF)
    {
        case SSI_SSIENR_OFFSET:
        {
            if(data == 0) // disabled
            {
                while(!ssiRx.empty())
                    ssiRx.pop();
            }
            break;
        }
        case SSI_DR0_OFFSET: // DR0
        {
            if(flashCmdOff)
            {
                if(flashCmd == 0x35) // read status2
                {
                    ssiRx.push(2);
                    return;
                }
                else if(flashCmd == 0x4B) // read uid
                {
                    static const uint8_t uid[]{'E', 0xDA, 0xF7, 0x20, 0x40, 'F', 'A', 'K'};
                    if(flashCmdOff < 4)
                        ssiRx.push(0);
                    else if(flashCmdOff <= 12)
                        ssiRx.push(uid[flashCmdOff - 4]);

                    flashCmdOff++;
                    return;
                }

                if(flashCmdOff >= 4) // done addr
                {
                    if(flashCmd == 2) // write
                    {
                        qspiFlash[flashAddr + (flashCmdOff - 4)] = data & 0xFF;
                        ssiRx.push(0);
                    }
                    else if(flashCmd == 3) // read
                        ssiRx.push(qspiFlash[flashAddr + (flashCmdOff - 4)]);
                }
                else
                {
                    // get addr
                    flashAddr |= data << (24 - (flashCmdOff * 8));
                    ssiRx.push(0);
                }
                flashCmdOff++;

                if(flashCmdOff == 4)
                {
                    if(flashCmd == 0x20/*4k erase*/)
                    {
                        memset(qspiFlash + flashAddr, 0xFF, 4 * 1024);
                        flashCmdOff = 0; // done
                    }
                }
            }
            else
            {
                flashCmd = data;
                if(flashCmd == 2) // write
                {
                    flashAddr = 0;
                    flashCmdOff++;
                }
                else if(flashCmd == 3) // read
                {
                    flashAddr = 0;
                    flashCmdOff++;
                }
                else if(flashCmd == 5) // read status
                {}
                else if(flashCmd == 6) // write enable
                {}
                else if(flashCmd == 0x20) // 4k erase
                {
                    flashAddr = 0;
                    flashCmdOff++;
                }
                else if(flashCmd == 0x35) // read status2
                    flashCmdOff++;
                else if(flashCmd == 0x4B) // unique id
                    flashCmdOff++;
                else if(flashCmd && flashCmd != 0xFF)
                    logf(LogLevel::NotImplemented, logComponent, "XIP SSI write %08X", data);

                ssiRx.push(0); //
            }

            return;
        }
    }
    logf(LogLevel::NotImplemented, logComponent, "XIP SSI W %s (%08X) = %08X", ssiRegNames[(addr & 0xFF) / 4], addr, data);
}

template<class T>
T MemoryBus::doSRAMRead(uint32_t addr) const
{
    // striped SRAM0-3, SRAM4-5
    // (SRAM0-3 is stored striped)
    if (addr < SRAM_END)
        return *reinterpret_cast<const T *>(sram + (addr & 0xFFFFF));

    logf(LogLevel::NotImplemented, logComponent, "SRAM R %08X", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doSRAMWrite(uint32_t addr, T data)
{
    if(addr < SRAM_END)
    {
        *reinterpret_cast<T *>(sram + (addr & 0xFFFFF)) = data;
        return;
    }

    logf(LogLevel::NotImplemented, logComponent, "SRAM W %08X = %08X", addr, data);
}

static const char *apbPeriphNames[]{
    "SYSINFO",
    "SYSCFG",
    "CLOCKS",
    "RESETS",
    "PSM",
    "IO_BANK0",
    "IO_QSPI",
    "PADS_BANK0",
    "PADS_QSPI",
    "XOSC",
    "PLL_SYS",
    "PLL_USB",
    "BUSCTRL",
    "UART0",
    "UART1",
    "SPI0",
    "SPI1",
    "I2C0",
    "I2C1",
    "ADC",
    "PWM",
    "TIMER",
    "WATCHDOG",
    "RTC",
    "ROSC",
    "VREG_AND_CHIP_RESET",
    "68000",
    "TBMAN"
};

// promote all reads to 32-bit
template<class T>
T MemoryBus::doAPBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    int shift = (addr & 3) * 8;
    return doAPBPeriphRead<uint32_t>(masterClock, addr & ~3) >> shift;
}

template<>
uint32_t MemoryBus::doAPBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    auto peripheral = static_cast<APBPeripheral>((addr >> 14) & 0x1F);
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case APBPeripheral::SysInfo:
            if(periphAddr == SYSINFO_GITREF_RP2040_OFFSET)
                return 0xDF2040DF; // TODO: put a real value here?
            break;

        case APBPeripheral::Clocks:
            return clocks.regRead(periphAddr);

        case APBPeripheral::Resets: // RESETS
        {
            // boot hack
            if(periphAddr == RESETS_RESET_DONE_OFFSET)
            {
                logf(LogLevel::NotImplemented, logComponent, "R RESET_DONE");
                return ~0u;
            }
            break;
        }

        case APBPeripheral::PSM:
        {
            if(periphAddr == PSM_FRCE_OFF_OFFSET)
                return psmOff;
            break;
        }

        case APBPeripheral::IO_Bank0:
            gpioUpdate(masterClock.getTime());
            return gpio.regRead(periphAddr);

        case APBPeripheral::IO_QSPI:
        {
            if(periphAddr < IO_QSPI_INTR_OFFSET)
            {
                int io = periphAddr / 8;
                if(periphAddr & 4)
                    return ioQSPICtrl[io];
                else // STATUS
                    return 0; // TODO
            }
            break;
        }

        case APBPeripheral::Pads_Bank0:
            gpioUpdate(masterClock.getTime());
            return gpio.padsRegRead(periphAddr);

        case APBPeripheral::PLL_Sys:
            return clocks.pllSysRegRead(periphAddr);
        case APBPeripheral::PLL_USB:
            return clocks.pllUSBRegRead(periphAddr);

        case APBPeripheral::UART0:
            return uart[0].regRead(periphAddr);
        case APBPeripheral::UART1:
            return uart[1].regRead(periphAddr);

        case APBPeripheral::SPI0:
        {
            if(periphAddr == SPI_SSPSR_OFFSET)
                return SPI_SSPSR_TNF_BITS;
            break;
        }

        case APBPeripheral::I2C0:
            return i2c[0].regRead(periphAddr);
        case APBPeripheral::I2C1:
            return i2c[1].regRead(periphAddr);

        case APBPeripheral::PWM:
            return pwm.regRead(masterClock.getTime(), periphAddr);

        case APBPeripheral::Timer:
            return timer.regRead(masterClock.getTime(), periphAddr);

        case APBPeripheral::Watchdog:
            return watchdog.regRead(masterClock.getTime(), periphAddr);

        case APBPeripheral::RTC:
            if(periphAddr == RTC_CTRL_OFFSET)
                return rtcCtrl | (rtcCtrl & 1) << 1; // RTC_ACTIVE = RTC_ENABLE

            break;

        default:
            break;
    }

    logf(LogLevel::NotImplemented, logComponent, "APBP R %s %04X", apbPeriphNames[static_cast<int>(peripheral)], addr & 0x3FFF);

    return doOpenRead<uint32_t>(addr);
}

// replicate all writes to 32bit
template<>
void MemoryBus::doAPBPeriphWrite(ClockTarget &masterClock, uint32_t addr, uint8_t data)
{
    doAPBPeriphWrite<uint32_t>(masterClock, addr & ~3, data | data << 8 | data << 16 | data << 24);
}

template<>
void MemoryBus::doAPBPeriphWrite(ClockTarget &masterClock, uint32_t addr, uint16_t data)
{
    doAPBPeriphWrite<uint32_t>(masterClock, addr & ~3, data | data << 16);
}

template<class T>
void MemoryBus::doAPBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data)
{
    auto peripheral = static_cast<APBPeripheral>((addr >> 14) & 0x1F);
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case APBPeripheral::Clocks:
            clocks.regWrite(periphAddr, data);
            return;

        case APBPeripheral::PSM:
        {
            int atomic = periphAddr >> 12;
            int addr = periphAddr & 0xFFF;
            if(addr == PSM_FRCE_OFF_OFFSET)
            {
                logf(LogLevel::NotImplemented, logComponent, "PSM FRCE_OFF %i %08X", atomic, data);
                updateReg(psmOff, data, atomic);
                return;
            }
            break;
        }

        case APBPeripheral::IO_Bank0:
            gpioUpdate(masterClock.getTime());
            gpio.regWrite(periphAddr, data);
            return;

        case APBPeripheral::IO_QSPI:
        {
            if((periphAddr & 0xFFF) < IO_QSPI_INTR_OFFSET)
            {
                int atomic = periphAddr >> 12;
                int io = (periphAddr & 0xFFF) / 8;
                if(periphAddr & 4) // CTRL
                {
                    uint32_t newVal;
                    if(atomic == 0)
                        newVal = data;
                    else if(atomic == 1)
                        newVal = ioQSPICtrl[io] ^ data;
                    else if(atomic == 2)
                        newVal = ioQSPICtrl[io] | data;
                    else
                        newVal = ioQSPICtrl[io] & ~data;

                    if(io == 1) // SS ...
                    {
                        if(((newVal >> 8) & 3) != 2) // ... forced high (deselected)
                            flashCmdOff = 0;
                    }

                    ioQSPICtrl[io] = newVal;
                    return;
                }
            }
            break;
        }

        case APBPeripheral::Pads_Bank0:
            gpioUpdate(masterClock.getTime());
            gpio.padsRegWrite(periphAddr, data);
            return;

        case APBPeripheral::PLL_Sys:
            clocks.pllSysRegWrite(periphAddr, data);
            return;
        case APBPeripheral::PLL_USB:
            clocks.pllUSBRegWrite(periphAddr, data);
            return;

        case APBPeripheral::UART0:
            uart[0].regWrite(periphAddr, data);
            return;
        case APBPeripheral::UART1:
            uart[0].regWrite(periphAddr, data);
            return;

        case APBPeripheral::I2C0:
            i2c[0].regWrite(periphAddr, data);
            return;
        case APBPeripheral::I2C1:
            i2c[1].regWrite(periphAddr, data);
            return;

        case APBPeripheral::PWM:
            pwm.regWrite(masterClock.getTime(), periphAddr, data);
            return;

        case APBPeripheral::Timer:
            timer.regWrite(masterClock.getTime(), periphAddr, data);
            calcNextInterruptTime();
            return;

        case APBPeripheral::Watchdog:
            watchdog.regWrite(masterClock.getTime(), periphAddr, data);
            return;

        case APBPeripheral::RTC:
            if(periphAddr == RTC_CTRL_OFFSET)
            {
                rtcCtrl = data;
                return;
            }
            break;

        default:
            break;
    }

    logf(LogLevel::NotImplemented, logComponent, "APBP W %s %04X = %08X", apbPeriphNames[static_cast<int>(peripheral)], addr & 0x3FFF, data);
}

// promote all reads to 32-bit
template<class T>
T MemoryBus::doAHBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    if(addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
    {
        // ... except USB DPRAM, which DOES handle narrow access
        usb.update(masterClock.getTime());
        return *reinterpret_cast<const T *>(usb.getRAM() + (addr & 0xFFF));
    }

    int shift = (addr & 3) * 8;
    return doAHBPeriphRead<uint32_t>(masterClock, addr & ~3) >> shift;
}

template<>
uint32_t MemoryBus::doAHBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    auto peripheral = static_cast<AHBPeripheral>((addr >> 20) & 0xF);
    auto periphAddr = addr & 0xFFFF;

    // update DMA for any periph read
    // TODO: any access that DMA could affect, possibly filter by dest addrs
    dma.update(masterClock.getTime());

    switch(peripheral)
    {
        case AHBPeripheral::DMA:
            return dma.regRead(masterClock.getTime(), periphAddr);

        case AHBPeripheral::USB:
        {
            if(addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
            {
                usb.update(masterClock.getTime());
                return *reinterpret_cast<const uint32_t *>(usb.getRAM() + (addr & 0xFFF));
            }
            else if(addr >= USBCTRL_REGS_BASE)
                return usb.regRead(masterClock.getTime(), periphAddr);

            break;
        }
        
        case AHBPeripheral::PIO0:
            return pio[0].regRead(masterClock.getTime(), periphAddr);

        case AHBPeripheral::PIO1:
            return pio[1].regRead(masterClock.getTime(), periphAddr);

        case AHBPeripheral::XIPAux:
            logf(LogLevel::NotImplemented, logComponent, "AHBP XIP_AUX R %08X", addr);
            break;
    }

    return doOpenRead<uint32_t>(addr);
}

// replicate all writes to 32bit
template<>
void MemoryBus::doAHBPeriphWrite(ClockTarget &masterClock, uint32_t addr, uint8_t data)
{
    if(addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
    {
        usb.update(masterClock.getTime());
        usb.getRAM()[addr & 0xFFF] = data;
        usb.ramWrite(addr & 0xFFF);
        return;
    }

    doAHBPeriphWrite<uint32_t>(masterClock, addr & ~3, data | data << 8 | data << 16 | data << 24);
}

template<>
void MemoryBus::doAHBPeriphWrite(ClockTarget &masterClock, uint32_t addr, uint16_t data)
{
    if(addr >= USBCTRL_DPRAM_BASE && addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
    {
        usb.update(masterClock.getTime());
        *reinterpret_cast<uint16_t *>(usb.getRAM() + (addr & 0xFFF)) = data;
        usb.ramWrite(addr & 0xFFF);
        return;
    }

    doAHBPeriphWrite<uint32_t>(masterClock, addr & ~3, data | data << 16);
}

template<class T>
void MemoryBus::doAHBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data)
{
    auto peripheral = static_cast<AHBPeripheral>((addr >> 20) & 0xF);
    auto periphAddr = addr & 0xFFFF;

    switch(peripheral)
    {
        case AHBPeripheral::DMA:
            dma.regWrite(masterClock.getTime(), periphAddr, data);
            calcNextInterruptTime();
            return;

        case AHBPeripheral::USB:
        {
            if(addr < USBCTRL_DPRAM_BASE + USB_DPRAM_SIZE)
            {
                usb.update(masterClock.getTime());
                *reinterpret_cast<T *>(usb.getRAM() + (addr & 0xFFF)) = data;
                usb.ramWrite(addr & 0xFFF);
                return;
            }
            else if(addr >= USBCTRL_REGS_BASE)
            {
                usb.regWrite(masterClock.getTime(), periphAddr, data);
                return;
            }
            break;
        }
        
        case AHBPeripheral::PIO0:
        {
            pio[0].regWrite(masterClock.getTime(), periphAddr, data);
            return;
        }
        case AHBPeripheral::PIO1:
        {
            pio[1].regWrite(masterClock.getTime(), periphAddr, data);
            return;
        }

        case AHBPeripheral::XIPAux:
            logf(LogLevel::NotImplemented, logComponent, "AHBP XIP_AUX W %08X = %08X", addr, data);
            return;
    }
}

// promote all reads to 32-bit
template<class T>
T MemoryBus::doIOPORTRead(ClockTarget &masterClock, int core, uint32_t addr)
{
    int shift = (addr & 3) * 8;
    return doIOPORTRead<uint32_t>(masterClock, core, addr & ~3) >> shift;
}

template<>
uint32_t MemoryBus::doIOPORTRead(ClockTarget &masterClock, int core, uint32_t addr)
{
    switch(addr & 0xFFF)
    {
        case SIO_CPUID_OFFSET:
            return core;

        case SIO_GPIO_IN_OFFSET:
            gpioUpdate(masterClock.getTime());
            return gpio.getInputs(masterClock.getTime());

        case SIO_GPIO_HI_IN_OFFSET:
        {
            // boot hack
            logf(LogLevel::NotImplemented, logComponent, "R GPIO_HI_IN");
            return 2;
        }

        case SIO_FIFO_ST_OFFSET:
        {
            bool vld = !coreFIFO[1 - core].empty();
            bool rdy = !coreFIFO[core].full();
            return (vld ? SIO_FIFO_ST_VLD_BITS : 0) | (rdy ? SIO_FIFO_ST_RDY_BITS : 0); // TODO: error flags
        }
        case SIO_FIFO_RD_OFFSET:
            return coreFIFO[1 - core].popOrDefault();

        case SIO_DIV_UDIVIDEND_OFFSET:
        case SIO_DIV_SDIVIDEND_OFFSET:
            return dividend[core];
        case SIO_DIV_UDIVISOR_OFFSET:
        case SIO_DIV_SDIVISOR_OFFSET:
            return divisor[core];
        case SIO_DIV_QUOTIENT_OFFSET:
            dividerDirty[core] = false;
            return divQuot[core];
        case SIO_DIV_REMAINDER_OFFSET:
            return divRem[core];
        case SIO_DIV_CSR_OFFSET:
            return (dividerDirty[core] ? SIO_DIV_CSR_DIRTY_BITS : 0) | SIO_DIV_CSR_READY_BITS;

        case SIO_INTERP0_ACCUM0_OFFSET:
        case SIO_INTERP0_ACCUM1_OFFSET:
            return interpolator[core][0].accum[((addr & 0xFFF) - SIO_INTERP0_ACCUM0_OFFSET) / 4];

        case SIO_INTERP0_POP_LANE0_OFFSET:
        case SIO_INTERP0_POP_LANE1_OFFSET:
        case SIO_INTERP0_POP_FULL_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP0_POP_LANE0_OFFSET) / 4;
            auto result = interpolator[core][0].peek[lane];
            popInterpolator(interpolator[core][0]);
            return result;
        }

        case SIO_INTERP0_PEEK_LANE0_OFFSET:
        case SIO_INTERP0_PEEK_LANE1_OFFSET:
        case SIO_INTERP0_PEEK_FULL_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP0_PEEK_LANE0_OFFSET) / 4;
            return interpolator[core][0].peek[lane];
        }

        case SIO_INTERP0_ACCUM0_ADD_OFFSET:
        case SIO_INTERP0_ACCUM1_ADD_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP0_ACCUM0_ADD_OFFSET) / 4;
            return interpolator[core][0].add_raw[lane];
        }

        case SIO_INTERP1_ACCUM0_OFFSET:
        case SIO_INTERP1_ACCUM1_OFFSET:
            return interpolator[core][1].accum[((addr & 0xFFF) - SIO_INTERP1_ACCUM0_OFFSET) / 4];

        case SIO_INTERP1_POP_LANE0_OFFSET:
        case SIO_INTERP1_POP_LANE1_OFFSET:
        case SIO_INTERP1_POP_FULL_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP1_POP_LANE0_OFFSET) / 4;
            auto result = interpolator[core][1].peek[lane];

            popInterpolator(interpolator[core][1]);
            return result;
        }

        case SIO_INTERP1_PEEK_LANE0_OFFSET:
        case SIO_INTERP1_PEEK_LANE1_OFFSET:
        case SIO_INTERP1_PEEK_FULL_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP1_PEEK_LANE0_OFFSET) / 4;
            return interpolator[core][1].peek[lane];
        }

        case SIO_INTERP1_ACCUM0_ADD_OFFSET:
        case SIO_INTERP1_ACCUM1_ADD_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP1_ACCUM0_ADD_OFFSET) / 4;
            return interpolator[core][1].add_raw[lane];
        }

        case SIO_SPINLOCK0_OFFSET:
        case SIO_SPINLOCK1_OFFSET:
        case SIO_SPINLOCK2_OFFSET:
        case SIO_SPINLOCK3_OFFSET:
        case SIO_SPINLOCK4_OFFSET:
        case SIO_SPINLOCK5_OFFSET:
        case SIO_SPINLOCK6_OFFSET:
        case SIO_SPINLOCK7_OFFSET:
        case SIO_SPINLOCK8_OFFSET:
        case SIO_SPINLOCK9_OFFSET:
        case SIO_SPINLOCK10_OFFSET:
        case SIO_SPINLOCK11_OFFSET:
        case SIO_SPINLOCK12_OFFSET:
        case SIO_SPINLOCK13_OFFSET:
        case SIO_SPINLOCK14_OFFSET:
        case SIO_SPINLOCK15_OFFSET:
        case SIO_SPINLOCK16_OFFSET:
        case SIO_SPINLOCK17_OFFSET:
        case SIO_SPINLOCK18_OFFSET:
        case SIO_SPINLOCK19_OFFSET:
        case SIO_SPINLOCK20_OFFSET:
        case SIO_SPINLOCK21_OFFSET:
        case SIO_SPINLOCK22_OFFSET:
        case SIO_SPINLOCK23_OFFSET:
        case SIO_SPINLOCK24_OFFSET:
        case SIO_SPINLOCK25_OFFSET:
        case SIO_SPINLOCK26_OFFSET:
        case SIO_SPINLOCK27_OFFSET:
        case SIO_SPINLOCK28_OFFSET:
        case SIO_SPINLOCK29_OFFSET:
        case SIO_SPINLOCK30_OFFSET:
        case SIO_SPINLOCK31_OFFSET:
        {
            int lock = (addr & 0xFF) / 4;

            if(spinlocks & (1 << lock))
                return 0;

            spinlocks |= 1 << lock;
            return 1 << lock;
        }
    }

    logf(LogLevel::NotImplemented, logComponent, "IOPORT R %08X", addr);
    return doOpenRead<uint32_t>(addr);
}

// replicate all writes to 32bit
template<>
void MemoryBus::doIOPORTWrite(ClockTarget &masterClock, int core, uint32_t addr, uint8_t data)
{
    doIOPORTWrite<uint32_t>(masterClock, core, addr & ~3, data | data << 8 | data << 16 | data << 24);
}

template<>
void MemoryBus::doIOPORTWrite(ClockTarget &masterClock, int core, uint32_t addr, uint16_t data)
{
    doIOPORTWrite<uint32_t>(masterClock, core, addr & ~3, data | data << 16);
}

template<class T>
void MemoryBus::doIOPORTWrite(ClockTarget &masterClock, int core, uint32_t addr, T data)
{
    auto doDiv = [this, core]()
    {
        // TODO: should take 8 cycles
        if(!divisor[core])
            return; // what should this do?

        if(dividerSigned[core])
        {
            divQuot[core] = static_cast<int32_t>(dividend[core]) / static_cast<int32_t>(divisor[core]);
            divRem[core] = static_cast<int32_t>(dividend[core]) % static_cast<int32_t>(divisor[core]);
        }
        else
        {
            divQuot[core] = dividend[core] / divisor[core];
            divRem[core] = dividend[core] % divisor[core];
        }
    };

    switch(addr & 0xFFF)
    {
        case SIO_GPIO_OUT_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.setOutputs(data);
            return;
        case SIO_GPIO_OUT_SET_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.setOutputMask(data);
            return;
        case SIO_GPIO_OUT_CLR_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.clearOutputMask(data);
            return;
        case SIO_GPIO_OUT_XOR_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.xorOutputMask(data);
            return;

        case SIO_GPIO_OE_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.setOutputEnables(data);
            return;
        case SIO_GPIO_OE_SET_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.setOutputEnableMask(data);
            return;
        case SIO_GPIO_OE_CLR_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.clearOutputEnableMask(data);
            return;
        case SIO_GPIO_OE_XOR_OFFSET:
            gpioUpdate(masterClock.getTime());
            gpio.xorOutputEnableMask(data);
            return;

        case SIO_FIFO_WR_OFFSET:
            coreFIFO[core].pushIfNotFull(data);
            // at least one status flag got set here...
            setPendingIRQ(SIO_IRQ_PROC0 + core);
            return;

        case SIO_DIV_UDIVIDEND_OFFSET:
            dividend[core] = data;
            dividerDirty[core] = true;
            dividerSigned[core] = false;
            doDiv();
            return;
        case SIO_DIV_UDIVISOR_OFFSET:
            divisor[core] = data;
            dividerDirty[core] = true;
            dividerSigned[core] = false;
            doDiv();
            return;
        case SIO_DIV_SDIVIDEND_OFFSET:
            dividend[core] = data;
            dividerDirty[core] = true;
            dividerSigned[core] = true;
            doDiv();
            return;
        case SIO_DIV_SDIVISOR_OFFSET:
            divisor[core] = data;
            dividerDirty[core] = true;
            dividerSigned[core] = true;
            doDiv();
            return;
        case SIO_DIV_QUOTIENT_OFFSET:
            divQuot[core] = data;
            dividerDirty[core] = true;
            return;
        case SIO_DIV_REMAINDER_OFFSET:
            divRem[core] = data;
            dividerDirty[core] = true;
            return;

        case SIO_INTERP0_ACCUM0_OFFSET:
        case SIO_INTERP0_ACCUM1_OFFSET:
            interpolator[core][0].accum[((addr & 0xFFF) - SIO_INTERP0_ACCUM0_OFFSET) / 4] = data;
            updateInterpolatorResult(interpolator[core][0]);
            return;

        case SIO_INTERP0_BASE0_OFFSET:
        case SIO_INTERP0_BASE1_OFFSET:
        case SIO_INTERP0_BASE2_OFFSET:
            interpolator[core][0].base[((addr & 0xFFF) - SIO_INTERP0_BASE0_OFFSET) / 4] = data;
            updateInterpolatorResult(interpolator[core][0]);
            return;

        case SIO_INTERP0_CTRL_LANE0_OFFSET:
        case SIO_INTERP0_CTRL_LANE1_OFFSET:
        {
            int lane = ((addr & 0xFFF) - SIO_INTERP0_CTRL_LANE0_OFFSET) / 4;
            if(data & SIO_INTERP0_CTRL_LANE0_BLEND_BITS)
                logf(LogLevel::NotImplemented, logComponent, "Interp 0 lane %i blend mode", lane);
    
            interpolator[core][0].ctrl[lane] = data;
            updateInterpolatorResult(interpolator[core][0]);
            return;
        }

        // TODO: ACCUMx_ADD
        // TODO: BASE_1AND0

        case SIO_INTERP1_ACCUM0_OFFSET:
        case SIO_INTERP1_ACCUM1_OFFSET:
            interpolator[core][1].accum[((addr & 0xFFF) - SIO_INTERP1_ACCUM0_OFFSET) / 4] = data;
            updateInterpolatorResult(interpolator[core][1]);
            return;

        case SIO_INTERP1_BASE0_OFFSET:
        case SIO_INTERP1_BASE1_OFFSET:
        case SIO_INTERP1_BASE2_OFFSET:
            interpolator[core][1].base[((addr & 0xFFF) - SIO_INTERP1_BASE0_OFFSET) / 4] = data;
            updateInterpolatorResult(interpolator[core][1]);
            return;

        case SIO_INTERP1_CTRL_LANE0_OFFSET:
        case SIO_INTERP1_CTRL_LANE1_OFFSET:
            interpolator[core][1].ctrl[((addr & 0xFFF) - SIO_INTERP1_CTRL_LANE0_OFFSET) / 4] = data;
            updateInterpolatorResult(interpolator[core][1]);
            return;

        // TODO: ACCUMx_ADD
        // TODO: BASE_1AND0

        case SIO_SPINLOCK0_OFFSET:
        case SIO_SPINLOCK1_OFFSET:
        case SIO_SPINLOCK2_OFFSET:
        case SIO_SPINLOCK3_OFFSET:
        case SIO_SPINLOCK4_OFFSET:
        case SIO_SPINLOCK5_OFFSET:
        case SIO_SPINLOCK6_OFFSET:
        case SIO_SPINLOCK7_OFFSET:
        case SIO_SPINLOCK8_OFFSET:
        case SIO_SPINLOCK9_OFFSET:
        case SIO_SPINLOCK10_OFFSET:
        case SIO_SPINLOCK11_OFFSET:
        case SIO_SPINLOCK12_OFFSET:
        case SIO_SPINLOCK13_OFFSET:
        case SIO_SPINLOCK14_OFFSET:
        case SIO_SPINLOCK15_OFFSET:
        case SIO_SPINLOCK16_OFFSET:
        case SIO_SPINLOCK17_OFFSET:
        case SIO_SPINLOCK18_OFFSET:
        case SIO_SPINLOCK19_OFFSET:
        case SIO_SPINLOCK20_OFFSET:
        case SIO_SPINLOCK21_OFFSET:
        case SIO_SPINLOCK22_OFFSET:
        case SIO_SPINLOCK23_OFFSET:
        case SIO_SPINLOCK24_OFFSET:
        case SIO_SPINLOCK25_OFFSET:
        case SIO_SPINLOCK26_OFFSET:
        case SIO_SPINLOCK27_OFFSET:
        case SIO_SPINLOCK28_OFFSET:
        case SIO_SPINLOCK29_OFFSET:
        case SIO_SPINLOCK30_OFFSET:
        case SIO_SPINLOCK31_OFFSET:
        {
            int lock = (addr & 0xFF) / 4;

            spinlocks &=  ~(1 << lock);
            return;
        }
    }
    logf(LogLevel::NotImplemented, logComponent, "IOPORT W %08X = %08X", addr, data);
}

template<class T>
T MemoryBus::doCPUInternalRead(ARMv6MCore &cpu, uint32_t addr) const
{
    int shift = (addr & 3) * 8;
    return cpu.readReg(addr & ~3) >> shift;
}

// replicate all writes to 32bit
template<>
void MemoryBus::doCPUInternalWrite(ARMv6MCore &cpu, uint32_t addr, uint8_t data)
{
    cpu.writeReg(addr & ~3, data | data << 8 | data << 16 | data << 24);
}

template<>
void MemoryBus::doCPUInternalWrite(ARMv6MCore &cpu, uint32_t addr, uint16_t data)
{
    cpu.writeReg(addr & ~3, data | data << 16);
}

template<class T>
void MemoryBus::doCPUInternalWrite(ARMv6MCore &cpu, uint32_t addr, T data)
{
    cpu.writeReg(addr, data);
}

template<class T>
T MemoryBus::doOpenRead(uint32_t addr) const
{
    return static_cast<T>(0xBADADD55); // TODO
}
