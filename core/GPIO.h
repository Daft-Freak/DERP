#pragma once

#include <cstdint>
#include <fstream>
#include <functional>

#include "hardware/structs/iobank0.h"

#include "ClockTarget.h"

class MemoryBus;

class GPIO final
{
public:
    using ReadCallback = std::function<void(uint64_t, GPIO &)>;
    using UpdateCallback = std::function<void(uint64_t, GPIO &, uint32_t)>;

    enum class Function
    {
        JTAG = 0, // 0-3
        SPI,
        UART,
        I2C,
        PWM,
        SIO,
        PIO0,
        PIO1,
        Clock, // 20-25
        USB,

        Count
    };

    GPIO(MemoryBus &mem);

    void reset();

    void update(uint64_t target);

    uint32_t getInputs(uint64_t time)
    {
        if(readCallback)
            readCallback(time, *this);

        return inputsToPeriph;
    }

    void setInputs(uint32_t inputs);

    void setInputMask(uint32_t mask) {setInputs(inputs | mask);}
    void clearInputMask(uint32_t mask) {setInputs(inputs & ~mask);}

    void setInputsFloating(uint32_t inputs);

    void setInputFloatingMask(uint32_t mask) {setInputsFloating(inputsFloating | mask);}
    void clearInputFloatingMask(uint32_t mask) {setInputsFloating(inputsFloating & ~mask);}

    void setFuncOutputs(Function func, uint32_t outputs);
    void setOutputs(uint32_t outputs);

    void setOutputMask(uint32_t mask) {setOutputs(outputs[int(Function::SIO)] | mask);}
    void clearOutputMask(uint32_t mask) {setOutputs(outputs[int(Function::SIO)] & ~mask);}
    void xorOutputMask(uint32_t mask) {setOutputs(outputs[int(Function::SIO)] ^ mask);}

    void setFuncOutputEnables(Function func, uint32_t outputs);
    void setOutputEnables(uint32_t outputs);

    void setOutputEnableMask(uint32_t mask) {setOutputEnables(outputEnables[int(Function::SIO)] | mask);}
    void clearOutputEnableMask(uint32_t mask) {setOutputEnables(outputEnables[int(Function::SIO)] & ~mask);}
    void xorOutputEnableMask(uint32_t mask) {setOutputEnables(outputEnables[int(Function::SIO)] ^ mask);}

    bool interruptsEnabledOnPin(int pin);

    void setReadCallback(ReadCallback cb);
    void setUpdateCallback(UpdateCallback cb);

    uint32_t getPadState() const {return padState;}

    void openLogFile(const char *filename);
    void closeLogFile();

    // IO_BANK0
    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    // PADS_BANK0
    uint32_t padsRegRead(uint32_t addr);
    void padsRegWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    void updateInterrupts(uint32_t oldInputs);
    void updateInputs();
    void updateOutputs();
    void updateOutputEnables();
    void updatePads();

    MemoryBus &mem;

    ClockTarget clock;

    iobank0_hw_t io;

    uint32_t padControl[NUM_BANK0_GPIOS + 2]; // GPIO0-29, SWCLK, SWD

    uint32_t inputs, inputsFloating; // external state

    // from peripheral
    uint32_t outputs[int(Function::Count)];
    uint32_t outputEnables[int(Function::Count)];

    uint32_t functionMask[int(Function::Count)];

    uint32_t inputsFromPad, inputsToPeriph;

    uint32_t outputsFromPeriph, outputsToPad, padState;
    uint32_t oeFromPeriph, oeToPad;

    ReadCallback readCallback;
    UpdateCallback updateCallback;

    std::ofstream logFile;
};