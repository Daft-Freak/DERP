#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/pads_bank0.h"

#include "GPIO.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::GPIO;

GPIO::GPIO(MemoryBus &mem) : mem(mem)
{
}

void GPIO::reset()
{
    for(auto &reg : ctrl)
        reg = IO_BANK0_GPIO0_CTRL_RESET;

    for(auto &reg : interrupts)
        reg = IO_BANK0_INTR0_RESET;

    for(auto &reg : proc0InterruptEnables)
        reg = IO_BANK0_PROC0_INTE0_RESET;

    for(auto &reg : padControl)
        reg = PADS_BANK0_GPIO0_RESET;

    padControl[30] = PADS_BANK0_SWCLK_RESET;
    padControl[31] = PADS_BANK0_SWD_RESET;

    inputs = 0;
    outputs = 0;
}

void GPIO::setInputs(uint32_t inputs)
{
    // TODO: handle floating inputs?

    uint32_t changed = inputs ^ this->inputs;

    if(changed)
    {
        for(int i = 0; i < 32; i++)
        {
            if(!(changed & (1 << i)))
                continue;

            // TODO: proc1
            int ioShift = i % 8 * 4;
            auto p0IntEn = (proc0InterruptEnables[i / 8] >> ioShift) & 0xF;

            bool newState = inputs & (1 << i);
            bool oldState = this->inputs & (1 << i);

            // TODO: level should stick
            if(!newState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_LEVEL_LOW_BITS))
                interrupts[i / 8] |= 1 << (ioShift + 0);
            else if(newState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_LEVEL_HIGH_BITS))
                interrupts[i / 8] |= 1 << (ioShift + 1);
            else if(!newState && oldState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_EDGE_LOW_BITS))
                interrupts[i / 8] |= 1 << (ioShift + 2);
            else if(newState && !oldState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_EDGE_HIGH_BITS))
                interrupts[i / 8] |= 1 << (ioShift + 3);

            if(interrupts[i / 8] & proc0InterruptEnables[i / 8])
            {
                // TODO: only to one core
                mem.setPendingIRQ(IO_IRQ_BANK0);
            }
        }
    }

    this->inputs = inputs;
}

void GPIO::setOutputs(uint32_t outputs)
{
    if(this->outputs == outputs)
        return;

    logf(LogLevel::Debug, logComponent, "out %08X", outputs);
    this->outputs = outputs;
}

bool GPIO::interruptsEnabledOnPin(int pin)
{
    // TODO: proc1
    int ioShift = pin % 8 * 4;
    auto p0IntEn = (proc0InterruptEnables[pin / 8] >> ioShift) & 0xF;

    return p0IntEn != 0;
}

void GPIO::setReadCallback(ReadCallback cb)
{
    readCallback = cb;
}

uint32_t GPIO::regRead(uint32_t addr)
{
    if(addr < IO_BANK0_INTR0_OFFSET)
    {
        if(addr & 4) // GPIOx_CTRL
            return ctrl[addr / 8];
        // else status
    }
    else if(addr < IO_BANK0_PROC0_INTE0_OFFSET) // INTR0-3
        return interrupts[(addr - IO_BANK0_INTR0_OFFSET) / 4];
    else if(addr < IO_BANK0_PROC0_INTF0_OFFSET) // PROC0_INTE0-3
        return proc0InterruptEnables[(addr - IO_BANK0_PROC0_INTE0_OFFSET) / 4];
    else if(addr >= IO_BANK0_PROC0_INTS0_OFFSET && addr < IO_BANK0_PROC1_INTE0_OFFSET) // PROC0_INTS0-3
    {
        int index = (addr - IO_BANK0_PROC0_INTS0_OFFSET) / 4;
        // TODO: force
        return interrupts[index] & proc0InterruptEnables[index];
    }

    logf(LogLevel::NotImplemented, logComponent, "IO_BANK0 R %04X", addr);

    return 0xBADADD55;
}

void GPIO::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr < IO_BANK0_INTR0_OFFSET)
    {
        if(addr & 4) // GPIOx_CTRL
        {
            updateReg(ctrl[addr / 8], data, atomic);
            return;
        }
        // else status
    }
    else if(addr < IO_BANK0_PROC0_INTE0_OFFSET) // INTR0-3
    {
        if(atomic == 0)
        {
            interrupts[(addr - IO_BANK0_INTR0_OFFSET) / 4] &= ~(data & 0xCCCCCCCC); // level can't be cleared
            return;
        }
    }
    else if(addr < 0x110) // PROC0_INTE0-3
    {
        updateReg(proc0InterruptEnables[(addr - IO_BANK0_PROC0_INTE0_OFFSET) / 4], data, atomic);
        return;
    }

    logf(LogLevel::NotImplemented, logComponent, "IO_BANK0 W %04X%s%08X", addr, op[atomic], data);
}

uint32_t GPIO::padsRegRead(uint32_t addr)
{
    if(addr == PADS_BANK0_VOLTAGE_SELECT_OFFSET)
    {
        logf(LogLevel::NotImplemented, logComponent, "PADS_BANK0 R VOLTAGE_SELECT");
    }
    else if(addr <= PADS_BANK0_SWD_OFFSET)
    {
        int gpio = addr / 4 - 1;
        return padControl[gpio];
    }
    else
        logf(LogLevel::Error, logComponent, "PADS_BANK0 R %04X", addr);

    return 0xBADADD55;
}

void GPIO::padsRegWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    if(addr == PADS_BANK0_VOLTAGE_SELECT_OFFSET)
    {
        logf(LogLevel::NotImplemented, logComponent, "PADS_BANK0 VOLTAGE_SELECT%s%08X", op[atomic], data);
    }
    else if(addr <= PADS_BANK0_SWD_OFFSET)
    {
        int gpio = addr / 4 - 1;
        if(updateReg(padControl[gpio], data, atomic))
            logf(LogLevel::NotImplemented, logComponent, "PADS_BANK0 GPIO%i%s%02X", gpio, op[atomic], data);
    }
    else
        logf(LogLevel::Error, logComponent, "PADS_BANK0 W %04X%s%08X", addr, op[atomic], data);
}