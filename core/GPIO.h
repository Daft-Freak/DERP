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
    using ReadCallback = std::function<uint32_t(uint64_t, uint32_t)>;

    GPIO(MemoryBus &mem);

    void reset();

    void update(uint64_t target);

    uint32_t getInputs(uint64_t time) const
    {
        if(readCallback)
            return readCallback(time, inputs);

        return inputs;
    }

    void setInputs(uint32_t inputs);

    void setInputMask(uint32_t mask) {setInputs(inputs | mask);}
    void clearInputMask(uint32_t mask) {setInputs(inputs & ~mask);}

    void setOutputs(uint32_t outputs);

    void setOutputMask(uint32_t mask) {setOutputs(outputs | mask);}
    void clearOutputMask(uint32_t mask) {setOutputs(outputs & ~mask);}
    void xorOutputMask(uint32_t mask) {setOutputs(outputs ^ mask);}

    bool interruptsEnabledOnPin(int pin);

    void setReadCallback(ReadCallback cb);

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
    void updateOutputs();
    void updatePads();

    MemoryBus &mem;

    ClockTarget clock;

    iobank0_hw_t io;

    uint32_t padControl[NUM_BANK0_GPIOS + 2]; // GPIO0-29, SWCLK, SWD

    uint32_t inputs;
    uint32_t outputs; // SIO outputs

    uint32_t outputsFromPeriph, outputsToPad, padState;

    ReadCallback readCallback;

    std::ofstream logFile;
};