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
    for(auto &io: io.io)
        io.ctrl = IO_BANK0_GPIO0_CTRL_RESET;

    for(auto &reg : io.intr)
        reg = IO_BANK0_INTR0_RESET;

    for(auto &reg : io.proc0_irq_ctrl.inte)
        reg = IO_BANK0_PROC0_INTE0_RESET;

    for(auto &reg : padControl)
        reg = PADS_BANK0_GPIO0_RESET;

    padControl[30] = PADS_BANK0_SWCLK_RESET;
    padControl[31] = PADS_BANK0_SWD_RESET;

    inputs = 0;
    inputsFloating = ~0u;
    outputs = 0;

    inputsFromPad = inputsToPeriph = 0;

    outputsFromPeriph = outputsToPad = padState = 0;
    oeFromPeriph = oeToPad = 0;
}

void GPIO::update(uint64_t target)
{
    auto passed = clock.getCyclesToTime(target);

    if(logFile.is_open())
    {
        uint32_t data[]{padState, passed};
        logFile.write(reinterpret_cast<char *>(data), sizeof(data));
    }

    clock.addCycles(passed);
}

void GPIO::setInputs(uint32_t inputs)
{
    if(this->inputs == inputs)
        return;

    auto oldInputs = inputsToPeriph;
    this->inputs = inputs;
    updateInputs();
    updateInterrupts(oldInputs);
    updatePads();
}

void GPIO::setInputsFloating(uint32_t inputsFloating)
{
    if(this->inputsFloating == inputsFloating)
        return;

    auto oldInputs = inputsToPeriph;
    this->inputsFloating = inputsFloating;
    updateInputs();
    updateInterrupts(oldInputs);
    updatePads();
}

void GPIO::setOutputs(uint32_t outputs)
{
    if(this->outputs == outputs)
        return;

    this->outputs = outputs;

    updateOutputs();
}

void GPIO::setOutputEnables(uint32_t outputEnables)
{
    if(this->outputEnables == outputEnables)
        return;

    this->outputEnables = outputEnables;

    updateOutputEnables();
}

bool GPIO::interruptsEnabledOnPin(int pin)
{
    // TODO: proc1
    int ioShift = pin % 8 * 4;
    auto p0IntEn = (io.proc0_irq_ctrl.inte[pin / 8] >> ioShift) & 0xF;

    return p0IntEn != 0;
}

void GPIO::setReadCallback(ReadCallback cb)
{
    readCallback = cb;
}

void GPIO::openLogFile(const char *filename)
{
    logFile.open(filename, std::fstream::out | std::fstream::binary);
}

void GPIO::closeLogFile()
{
    logFile.close();
}

uint32_t GPIO::regRead(uint32_t addr)
{
    if(addr < IO_BANK0_INTR0_OFFSET)
    {
        if(addr & 4) // GPIOx_CTRL
            return io.io[addr / 8].ctrl;
        // else status ()
    }
    else if(addr < IO_BANK0_PROC0_INTE0_OFFSET) // INTR0-3
        return io.intr[(addr - IO_BANK0_INTR0_OFFSET) / 4];
    else if(addr < IO_BANK0_PROC0_INTF0_OFFSET) // PROC0_INTE0-3
        return io.proc0_irq_ctrl.inte[(addr - IO_BANK0_PROC0_INTE0_OFFSET) / 4];
    else if(addr >= IO_BANK0_PROC0_INTS0_OFFSET && addr < IO_BANK0_PROC1_INTE0_OFFSET) // PROC0_INTS0-3
    {
        int index = (addr - IO_BANK0_PROC0_INTS0_OFFSET) / 4;
        // TODO: force
        return io.intr[index] & io.proc0_irq_ctrl.inte[index];
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
            if(updateReg(io.io[addr / 8].ctrl, data, atomic))
                updateOutputs();
            return;
        }
        // else status (read-only)
    }
    else if(addr < IO_BANK0_PROC0_INTE0_OFFSET) // INTR0-3
    {
        if(atomic == 0)
        {
            io.intr[(addr - IO_BANK0_INTR0_OFFSET) / 4] &= ~(data & 0xCCCCCCCC); // level can't be cleared
            return;
        }
    }
    else if(addr < 0x110) // PROC0_INTE0-3
    {
        updateReg(io.proc0_irq_ctrl.inte[(addr - IO_BANK0_PROC0_INTE0_OFFSET) / 4], data, atomic);
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
        {
            logf(LogLevel::NotImplemented, logComponent, "PADS_BANK0 GPIO%i%s%02X", gpio, op[atomic], data);
            updateInputs();
            updatePads();
        }
    }
    else
        logf(LogLevel::Error, logComponent, "PADS_BANK0 W %04X%s%08X", addr, op[atomic], data);
}

void GPIO::updateInterrupts(uint32_t oldInputs)
{
    for(unsigned i = 0; i < NUM_BANK0_GPIOS; i++)
    {
        // TODO: proc1
        int ioShift = i % 8 * 4;
        auto p0IntEn = (io.proc0_irq_ctrl.inte[i / 8] >> ioShift) & 0xF;

        bool newState = inputsToPeriph & (1 << i);
        bool oldState = oldInputs & (1 << i);

        // TODO: IRQOVER

        // TODO: level should stick
        if(!newState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_LEVEL_LOW_BITS))
            io.intr[i / 8] |= 1 << (ioShift + 0);
        else if(newState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_LEVEL_HIGH_BITS))
            io.intr[i / 8] |= 1 << (ioShift + 1);
        else if(!newState && oldState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_EDGE_LOW_BITS))
            io.intr[i / 8] |= 1 << (ioShift + 2);
        else if(newState && !oldState && (p0IntEn & IO_BANK0_PROC0_INTE0_GPIO0_EDGE_HIGH_BITS))
            io.intr[i / 8] |= 1 << (ioShift + 3);

        if(io.intr[i / 8] & io.proc0_irq_ctrl.inte[i / 8])
        {
            // TODO: only to one core
            mem.setPendingIRQ(IO_IRQ_BANK0);
        }
    }
}

void GPIO::updateInputs()
{
    inputsFromPad = inputs & ~inputsFloating;

    // pulls
    for(unsigned i = 0; i < NUM_BANK0_GPIOS; i++)
    {
        if(!(inputsFloating & (1 << i)))
            continue;

        if(padControl[i] & PADS_BANK0_GPIO0_PUE_BITS)
            inputsFromPad |= (1 << i);
        // else PDE
        // it's already 0
    }

    // TODO: overrides
    inputsToPeriph = inputsFromPad;
}

void GPIO::updateOutputs()
{
    // TODO: func sel
    outputsFromPeriph = outputs;

    // TODO: overrides
    outputsToPad = outputsFromPeriph;

    updatePads();
}

void GPIO::updateOutputEnables()
{
    // TODO: func sel
    oeFromPeriph = outputEnables;

    // TODO: overrides
    oeToPad = oeFromPeriph;

    updatePads();
}

void GPIO::updatePads()
{
    // TODO: output disable
    padState = outputsToPad & oeToPad;

    // make the inputs visible if no output
    // TODO: input enable?
    padState |= inputsFromPad & ~oeToPad;
}