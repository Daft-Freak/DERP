#include <cassert>
#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

#include "ARMv6MCore.h"

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

enum MemoryRegion
{
    Region_ROM         = 0x00,
    Region_XIP         = 0x10,
    Region_XIP_NoAlloc = 0x11,
    Region_XIP_NoCache = 0x12,
    Region_XIP_NoCNoA  = 0x13,
    Region_XIP_SSI     = 0x18,
    Region_SRAM        = 0x20,
    Region_APBPeriph   = 0x40,
    Region_AHBPeriph   = 0x50,
    Region_IOPORT      = 0xD0,
    Region_CPUInternal = 0xE0,
};

template uint8_t MemoryBus::read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential);
template uint16_t MemoryBus::read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential);
template uint32_t MemoryBus::read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential);
template void MemoryBus::write(BusMasterPtr master, uint32_t addr, uint8_t val, int &cycles, bool sequential);
template void MemoryBus::write(BusMasterPtr master, uint32_t addr, uint16_t val, int &cycles, bool sequential);
template void MemoryBus::write(BusMasterPtr master, uint32_t addr, uint32_t val, int &cycles, bool sequential);

static inline uint32_t getStripedSRAMAddr(uint32_t addr)
{
    int bank = (addr >> 2) & 3;
    int word = (addr >> 4) & 0xFFFF; // a few too many bits

    return bank * 64 * 1024 + word * 4 + (addr & 3);
}

MemoryBus::MemoryBus() : gpio(*this), timer(*this), dma(*this)
{
    clocks.addClockTarget(4/*REF*/, watchdog.getClock());

    clocks.addClockTarget(5/*SYS*/, dma.getClock());
}

void MemoryBus::setBootROM(const uint8_t *rom)
{
    bootROM = rom;
}

void MemoryBus::setCPU(ARMv6MCore *cpu)
{
    this->cpu = cpu;
}

void MemoryBus::reset()
{
    clocks.reset();
    gpio.reset();
    timer.reset();
    watchdog.reset();

    dma.reset();
}

template<class T>
T MemoryBus::read(BusMasterPtr master, uint32_t addr, int &cycles, bool sequential)
{
    auto masterClock = std::holds_alternative<ARMv6MCore *>(master) ?
        std::get<ARMv6MCore *>(master)->getClock() :
        std::get<DMA *>(master)->getClock();

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

        case Region_XIP_SSI:
            accessCycles(1);
            return doXIPSSIRead<T>(addr);

        case Region_SRAM:
            accessCycles(1);
            return doSRAMRead<T>(addr);

        case Region_APBPeriph:
            accessCycles(3);
            return doAPBPeriphRead<T>(masterClock, addr);

        case Region_AHBPeriph:
            accessCycles(1);
            return doAHBPeriphRead<T>(masterClock, addr);

        case Region_IOPORT:
            return doIOPORTRead<T>(addr);

        case Region_CPUInternal:
            accessCycles(1);
            assert(std::holds_alternative<ARMv6MCore *>(master));
            return doCPUInternalRead<T>(*std::get<ARMv6MCore *>(master), addr);
    }

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::write(BusMasterPtr master, uint32_t addr, T data, int &cycles, bool sequential)
{
    auto masterClock = std::holds_alternative<ARMv6MCore *>(master) ?
        std::get<ARMv6MCore *>(master)->getClock() :
        std::get<DMA *>(master)->getClock();

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

        case Region_APBPeriph:
            accessCycles(4);
            doAPBPeriphWrite<T>(masterClock, addr, data);
            return;

        case Region_AHBPeriph:
            accessCycles(1);
            doAHBPeriphWrite<T>(masterClock, addr, data);
            return;

        case Region_IOPORT:
            doIOPORTWrite<T>(addr, data);
            return;

        case Region_CPUInternal:
            accessCycles(1);
            assert(std::holds_alternative<ARMv6MCore *>(master));
            doCPUInternalWrite<T>(*std::get<ARMv6MCore *>(master), addr, data);
            return;
    }
}

const uint8_t *MemoryBus::mapAddress(uint32_t addr) const
{
    switch(addr >> 24)
    {
        case Region_ROM:
            return bootROM ? bootROM + (addr & 0x3FFF) : nullptr;

        case Region_XIP:
            return qspiFlash + (addr & 0xFFFFFF);

        case Region_SRAM:
        {
            // SRAM0-3 is stored striped so that we can do this
            // + SRAM4-5
            if(addr < 0x20042000)
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

        case Region_SRAM:
        {
            // SRAM0-3 (striped) + SRAM4-5
            if(addr < 0x20042000)
                return sram + (addr & 0xFFFFF);

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

void MemoryBus::updatePC(uint32_t pc)
{
    // TODO: could be non-alloc cached
    pcInCachedXIP = (pc >> 24) == Region_XIP;
}

void MemoryBus::peripheralUpdate(uint64_t target)
{
    watchdog.update(target);
    timer.update(target);

    dma.update(target);
}

void MemoryBus::peripheralUpdate(uint64_t target, uint32_t irqMask)
{
    // only update things that might cause interrupts
    
    if(irqMask & (0xF)/*TIMER_IRQ0-3*/)
        timer.updateForInterrupts(target);

    if(irqMask & (1 << 11/*DMA_IRQ_0*/ | 1 << 12/*DMA_IRQ_1*/))
        dma.updateForInterrupts(target);
}

uint64_t MemoryBus::getNextInterruptTime(uint64_t target)
{
    // clamp to when there might be an interrupt
    // TODO: check cpu enabled mask

    target = timer.getNextInterruptTime(target);

    return target;
}

void MemoryBus::setPendingIRQ(int n)
{
    // TODO: and the other one
    cpu->setPendingIRQ(n);
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
    switch((addr & 0xFF) / 4)
    {
        case 8: // TXFLR
            return 0;
        case 9: // RXFLR
            return ssiRx.size();

        case 10: // SR
            return 1 << 1/*TFNF*/ | 1 << 2/*TFE*/ | (ssiRx.empty() ? 0 : 1 << 3/*RFNE*/);

        case 24: // DR0
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
    printf("XIP SSI R %s (%08X)\n", ssiRegNames[(addr & 0xFF) / 4], addr);
    return 0;
}

template<class T>
void MemoryBus::doXIPSSIWrite(uint32_t addr, T data)
{
    switch((addr & 0xFF) / 4)
    {
        case 2: // SSIENA
        {
            if(data == 0) // disabled
            {
                while(!ssiRx.empty())
                    ssiRx.pop();
            }
            break;
        }
        case 24: // DR0
        {
            if(flashCmdOff)
            {
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
                else if(flashCmd == 6) // write enable
                {}
                else if(flashCmd == 0x20) // 4k erase
                {
                    flashAddr = 0;
                    flashCmdOff++;
                }
                else if(flashCmd)
                    printf("XIP SSI write %08X\n", data);

                ssiRx.push(0); //
            }

            return;
        }
    }
    printf("XIP SSI W %s (%08X) = %08X\n", ssiRegNames[(addr & 0xFF) / 4], addr, data);
}

template<class T>
T MemoryBus::doSRAMRead(uint32_t addr) const
{
    // striped SRAM0-3, SRAM4-5
    // (SRAM0-3 is stored striped)
    if (addr < 0x20042000)
        return *reinterpret_cast<const T *>(sram + (addr & 0xFFFFF));

    printf("SRAM R %08X\n", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doSRAMWrite(uint32_t addr, T data)
{
    if(addr < 0x20042000)
    {
        *reinterpret_cast<T *>(sram + (addr & 0xFFFFF)) = data;
        return;
    }

    printf("SRAM W %08X = %08X\n", addr, data);
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

template<class T>
T MemoryBus::doAPBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    auto peripheral = (addr >> 14) & 0x1F;
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case 2: // CLOCKS
            return clocks.regRead(periphAddr);

        case 3: // RESETS
        {
            // boot hack
            if(addr == 0x4000C008)
            {
                printf("R RESET_DONE\n");
                return ~0;
            }
            break;
        }

        case 5: // IO_BANK0
            return gpio.regRead(periphAddr);

        case 6: // IO_QSPI
        {
            if(periphAddr < 0x30)
            {
                int io = periphAddr / 8;
                if(periphAddr & 4)
                    return ioQSPICtrl[io];
                else // STATUS
                    return 0; // TODO
            }
            break;
        }

        case 7: // PADS_BANK0
            return gpio.padsRegRead(periphAddr);

        case 10: // PLL_SYS
            return clocks.pllSysRegRead(periphAddr);

        case 11: // PLL_USB
            return clocks.pllUSBRegRead(periphAddr);

        case 20: // PWM
        {
            if((periphAddr & 0xFFF) < 0xA0)
            {
                int chan = (periphAddr & 0xFFF) / 20;
                int reg = (periphAddr / 4) % 5;

                if(reg == 0)
                    return pwmCSR[chan];
                else if(reg == 1)
                    return pwmDIV[chan];
                else if(reg == 2)
                    return pwmCTR[chan];
                else if(reg == 3)
                    return pwmCC[chan];
                else
                    return pwmTOP[chan];
            }

            break;
        }

        case 21: // TIMER
            timer.update(masterClock.getTime());
            return timer.regRead(periphAddr);

        case 22: // WATCHDOG
            watchdog.update(masterClock.getTime());
            return watchdog.regRead(periphAddr);
    }

    printf("APBP R %s %04X\n", apbPeriphNames[peripheral], addr & 0x3FFF);

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doAPBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data)
{
    auto peripheral = (addr >> 14) & 0x1F;
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case 2: // CLOCKS
            clocks.regWrite(periphAddr, data);
            return;

        case 5: // IO_BANK0
            gpio.regWrite(periphAddr, data);
            return;

        case 6: // IO_QSPI
        {
            if((periphAddr & 0xFFF) < 0x30)
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

        case 7: // PADS_BANK0
            gpio.padsRegWrite(periphAddr, data);
            return;

        case 10: // PLL_SYS
            clocks.pllSysRegWrite(periphAddr, data);
            return;

        case 11: // PLL_USB
            clocks.pllUSBRegWrite(periphAddr, data);
            return;

        case 20: // PWM
        {
            if((periphAddr & 0xFFF) < 0xA0)
            {
                int atomic = periphAddr >> 12;
                int chan = (periphAddr & 0xFFF) / 20;
                int reg = (periphAddr / 4) % 5;

                if(reg == 0)
                {
                    updateReg(pwmCSR[chan], data, atomic);
                    return;
                }
                else if(reg == 1)
                {
                    updateReg(pwmDIV[chan], data, atomic);
                    return;
                }
                else if(reg == 2)
                {
                    updateReg(pwmCTR[chan], data, atomic);
                    return;
                }
                else if(reg == 3)
                {
                    updateReg(pwmCC[chan], data, atomic);
                    return;
                }
                else
                {
                    updateReg(pwmTOP[chan], data, atomic);
                    return;
                }
            }

            break;
        }

        case 21: // TIMER
            timer.update(masterClock.getTime());
            timer.regWrite(periphAddr, data);
            return;

        case 22: // WATCHDOG
            watchdog.update(masterClock.getTime());
            watchdog.regWrite(periphAddr, data);
            return;
    }

    printf("APBP W %s %04X = %08X\n", apbPeriphNames[peripheral], addr & 0x3FFF, data);
}

template<class T>
T MemoryBus::doAHBPeriphRead(ClockTarget &masterClock, uint32_t addr)
{
    if(addr < 0x50100000) // DMA
    {
        dma.update(masterClock.getTime());
        return dma.regRead(addr & 0xFFFF);
    }
    else if(addr < 0x50101000) // USB DPRAM
        return doRead<T>(usbDPRAM, addr);
    else if(addr >= 0x50200000 && addr < 0x50300000)
    {
        if(addr == 0x50200004) // FSTAT
            return T(0x0F000F00); // all FIFOs empty
        if(addr == 0x50200008) // FDEBUG
            return T(0xF << 24); // all TXSTALL

        printf("PIO0 R %08X\n", addr);
    }
    else
        printf("AHBP R %08X\n", addr);

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doAHBPeriphWrite(ClockTarget &masterClock, uint32_t addr, T data)
{
    if(addr < 0x50100000) // DMA
    {
        dma.update(masterClock.getTime());
        dma.regWrite(addr & 0xFFFF, data);
        return;
    }
    else if(addr < 0x50101000) // USB DPRAM
    {
        doWrite<T>(usbDPRAM, addr, data);
        return;
    }
    else if(addr >= 0x50200000 && addr < 0x50300000)
    {
        if(addr == 0x50200010) // TXF0
        {}
        else
            printf("PIO0 W %08X = %08X\n", addr, data);
    }
    else
        printf("AHBP W %08X = %08X\n", addr, data);
}

template<class T>
T MemoryBus::doIOPORTRead(uint32_t addr)
{
    switch(addr & 0xFFF)
    {
        case 0: // CPUID
            return 0; // TODO: need to know which CPU is reading

        case 4: // GPIO_IN
            return gpio.getInputs();

        case 8:  // GPIO_HI_IN
        {
            // boot hack
            printf("R GPIO_HI_IN\n");
            return 2;
        }

        case 0x60: // DIV_UDIVIDEND
        case 0x68: // DIV_SDIVIDEND
            return dividend;
        case 0x64: // DIV_UDIVISOR
        case 0x6C: // DIV_SDIVISOR
            return divisor;
        case 0x70: // DIV_QUOTIENT
            dividerDirty = false;
            return divQuot;
        case 0x74: // DIV_REMAINDER
            return divRem;
        case 0x78: // DIV_CSR
            return (dividerDirty ? 2 : 0) | 1;

        case 0x100: // SPINLOCK0
        case 0x104:
        case 0x108:
        case 0x10C:
        case 0x110:
        case 0x114:
        case 0x118:
        case 0x11C:
        case 0x120:
        case 0x124:
        case 0x128:
        case 0x12C:
        case 0x130:
        case 0x134:
        case 0x138:
        case 0x13C:
        case 0x140:
        case 0x144:
        case 0x148:
        case 0x14C:
        case 0x150:
        case 0x154:
        case 0x158:
        case 0x15C:
        case 0x160:
        case 0x164:
        case 0x168:
        case 0x16C:
        case 0x170:
        case 0x174:
        case 0x178:
        case 0x17C: // SPINLOCK31
        {
            int lock = (addr & 0xFF) / 4;

            if(spinlocks & (1 << lock))
                return 0;

            spinlocks |= 1 << lock;
            return T(1 << lock);
        }
    }

    printf("IOPORT R %08X\n", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doIOPORTWrite(uint32_t addr, T data)
{
    auto doDiv = [this]()
    {
        // TODO: should take 8 cycles
        if(!divisor)
            return; // what should this do?

        if(dividerSigned)
        {
            divQuot = static_cast<int32_t>(dividend) / divisor;
            divRem = static_cast<int32_t>(dividend) % divisor;
        }
        else
        {
            divQuot = dividend / divisor;
            divRem = dividend % divisor;
        }
    };

    switch(addr & 0xFFF)
    {
        case 0x60: // DIV_UDIVIDEND
            dividend = data;
            dividerDirty = true;
            dividerSigned = false;
            doDiv();
            return;
        case 0x64: // DIV_UDIVISOR
            divisor = data;
            dividerDirty = true;
            dividerSigned = false;
            doDiv();
            return;
        case 0x68: // DIV_SDIVIDEND
            dividend = data;
            dividerDirty = true;
            dividerSigned = true;
            doDiv();
            return;
        case 0x6C: // DIV_SDIVISOR
            divisor = data;
            dividerDirty = true;
            dividerSigned = true;
            doDiv();
            return;
        case 0x70: // DIV_QUOTIENT
            divQuot = data;
            dividerDirty = true;
            return;
        case 0x74: // DIV_REMAINDER
            divRem = data;
            dividerDirty = true;
            return;

        case 0x100: // SPINLOCK0
        case 0x104:
        case 0x108:
        case 0x10C:
        case 0x110:
        case 0x114:
        case 0x118:
        case 0x11C:
        case 0x120:
        case 0x124:
        case 0x128:
        case 0x12C:
        case 0x130:
        case 0x134:
        case 0x138:
        case 0x13C:
        case 0x140:
        case 0x144:
        case 0x148:
        case 0x14C:
        case 0x150:
        case 0x154:
        case 0x158:
        case 0x15C:
        case 0x160:
        case 0x164:
        case 0x168:
        case 0x16C:
        case 0x170:
        case 0x174:
        case 0x178:
        case 0x17C: // SPINLOCK31
        {
            int lock = (addr & 0xFF) / 4;

            spinlocks &=  ~(1 << lock);
            return;
        }
    }
    printf("IOPORT W %08X = %08X\n", addr, data);
}

template<class T>
T MemoryBus::doCPUInternalRead(ARMv6MCore &cpu, uint32_t addr) const
{
    return cpu.readReg(addr);
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
