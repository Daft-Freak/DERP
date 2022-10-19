#include <chrono> // timer hack
#include <cstdio>
#include <cstring>

#include "MemoryBus.h"

#include "ARMv6MCore.h"

enum MemoryRegion
{
    Region_ROM         = 0x00,
    Region_XIP         = 0x10,
    Region_XIP_SSI     = 0x18,
    Region_SRAM        = 0x20,
    Region_APBPeriph   = 0x40,
    Region_AHBPeriph   = 0x50,
    Region_IOPORT      = 0xD0,
    Region_CPUInternal = 0xE0,
};

template uint8_t MemoryBus::read(ARMv6MCore &cpu, uint32_t addr, int &cycles, bool sequential);
template uint16_t MemoryBus::read(ARMv6MCore &cpu, uint32_t addr, int &cycles, bool sequential);
template uint32_t MemoryBus::read(ARMv6MCore &cpu, uint32_t addr, int &cycles, bool sequential);
template void MemoryBus::write(ARMv6MCore &cpu, uint32_t addr, uint8_t val, int &cycles, bool sequential);
template void MemoryBus::write(ARMv6MCore &cpu, uint32_t addr, uint16_t val, int &cycles, bool sequential);
template void MemoryBus::write(ARMv6MCore &cpu, uint32_t addr, uint32_t val, int &cycles, bool sequential);

static inline uint32_t getStripedSRAMAddr(uint32_t addr)
{
    int bank = (addr >> 2) & 3;
    int word = (addr >> 4) & 0xFFFF; // a few too many bits

    return bank * 64 * 1024 + word * 4 + (addr & 3);
}

MemoryBus::MemoryBus() {}

void MemoryBus::setBootROM(const uint8_t *rom)
{
    bootROM = rom;
}

void MemoryBus::reset()
{

}

template<class T>
T MemoryBus::read(ARMv6MCore &cpu, uint32_t addr, int &cycles, bool sequential)
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
            accessCycles(1);
            return doRead<T>(qspiFlash, addr);

        case Region_XIP_SSI:
            accessCycles(1);
            return doXIPSSIRead<T>(addr);

        case Region_SRAM:
            accessCycles(1);
            return doSRAMRead<T>(addr);

        case Region_APBPeriph:
            accessCycles(4);
            return doAPBPeriphRead<T>(addr);

        case Region_AHBPeriph:
            accessCycles(1);
            return doAHBPeriphRead<T>(addr);

        case Region_IOPORT:
            accessCycles(1);
            return doIOPORTRead<T>(addr);

        case Region_CPUInternal:
            accessCycles(1);
            return doCPUInternalRead<T>(cpu, addr);
    }

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::write(ARMv6MCore &cpu, uint32_t addr, T data, int &cycles, bool sequential)
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

        case Region_APBPeriph:
            accessCycles(5);
            doAPBPeriphWrite<T>(addr, data);
            return;

        case Region_AHBPeriph:
            accessCycles(1);
            doAHBPeriphWrite<T>(addr, data);
            return;

        case Region_IOPORT:
            accessCycles(1);
            doIOPORTWrite<T>(addr, data);
            return;

        case Region_CPUInternal:
            accessCycles(1);
            doCPUInternalWrite<T>(cpu, addr, data);
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
            if(addr < 0x20040000) // striped SRAM0-3
                return nullptr;
            else
            {
                // SRAM4-5 (or OOB)
                if(addr < 0x20042000)
                    return sram + (addr & 0xFFFFF);
            }

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
            if(addr < 0x20040000) // striped SRAM0-3
                return nullptr; // can't map this for more than one word...
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
            return 1 << 2; // tx empty

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
        case 24: // DR0
        {
            if(flashCmdOff)
            {
                if(flashCmdOff >= 4)
                    ssiRx.push(qspiFlash[flashAddr + (flashCmdOff - 4)]);
                else
                {
                    // get read addr
                    flashAddr |= data << (24 - (flashCmdOff * 8));
                    ssiRx.push(0);
                }
                flashCmdOff++;
            }
            else
            {
                flashCmd = data;
                if(flashCmd == 3) // read
                {
                    flashAddr = 0;
                    flashCmdOff++;
                }
                else
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
    if(addr < 0x20040000) // striped SRAM0-3
        return *reinterpret_cast<const T *>(sram + getStripedSRAMAddr(addr));
    else if (addr < 0x20042000) // SRAM4-5
        return *reinterpret_cast<const T *>(sram + (addr & 0xFFFFF));

    printf("SRAM R %08X\n", addr);
    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doSRAMWrite(uint32_t addr, T data)
{
    if(addr < 0x20040000) // striped SRAM0-3
    {
        *reinterpret_cast<T *>(sram + getStripedSRAMAddr(addr)) = data;
        return;
    }
    else if (addr < 0x20042000) // SRAM4-5
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
T MemoryBus::doAPBPeriphRead(uint32_t addr) const
{
    auto peripheral = (addr >> 14) & 0x1F;
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case 2: // CLOCKS
        {
            // boot hack
            if(periphAddr == 0x38 || periphAddr == 0x44)
            {
                printf("R CLK_%s_SELECTED\n", periphAddr == 0x38 ? "REF" : "SYS");

                // will eventually return the right one
                static int i = 0;
                int ret = 1 << i;
                i = (i + 1) % 3;
                return ret;
            }
            break;
        }
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

        case 21: // TIMER
        {
            if(periphAddr == 0x24 || periphAddr == 0x28) // TIMERRAWH/L
            {
                auto time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                return periphAddr == 0x24 ? time >> 32 : time;
            }
        }
    }

    printf("APBP R %s %04X\n", apbPeriphNames[peripheral], addr & 0x3FFF);

    return doOpenRead<T>(addr);
}

template<class T>
void MemoryBus::doAPBPeriphWrite(uint32_t addr, T data)
{
    auto peripheral = (addr >> 14) & 0x1F;
    auto periphAddr = addr & 0x3FFF;

    switch(peripheral)
    {
        case 6: // IO_QSPI
        {
            if(periphAddr < 0x30)
            {
                int io = periphAddr / 8;
                if(periphAddr & 4) // CTRL
                {
                    if(io == 1 && ((ioQSPICtrl[io] >> 8) & 3) != 2 && ((data >> 8) & 3) == 2) // SS forced high -> low
                        flashCmdOff = 0;

                    ioQSPICtrl[io] = data;
                    return;
                }
            }
            break;
        }
    }

    printf("APBP W %s %04X = %08X\n", apbPeriphNames[peripheral], addr & 0x3FFF, data);
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

    printf("AHBP W %08X = %08X\n", addr, data);
}

template<class T>
T MemoryBus::doIOPORTRead(uint32_t addr)
{
    switch(addr & 0xFFF)
    {
        case 0: // CPUID
            return 0; // TODO: need to know which CPU is reading

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
            // boot hack
            // ... but also we don't have two cores yet
            int lock = (addr & 0xFF) / 4;
            printf("R SPINLOCK%i\n", lock);
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
