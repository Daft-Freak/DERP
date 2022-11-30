#pragma once

#include <cstdint>
#include <functional>

class MemoryBus;

class GPIO final
{
public:
    using ReadCallback = std::function<uint32_t(uint64_t, uint32_t)>;

    GPIO(MemoryBus &mem);

    void reset();

    uint32_t getInputs(uint64_t time) const
    {
        if(readCallback)
            return readCallback(time, inputs);

        return inputs;
    }

    void setInputs(uint32_t inputs);

    void setInputMask(uint32_t mask) {setInputs(inputs | mask);}
    void clearInputMask(uint32_t mask) {setInputs(inputs & ~mask);}

    bool interruptsEnabledOnPin(int pin);

    void setReadCallback(ReadCallback cb);

    // IO_BANK0
    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    // PADS_BANK0
    uint32_t padsRegRead(uint32_t addr);
    void padsRegWrite(uint32_t addr, uint32_t data);

private:
    MemoryBus &mem;

    uint32_t ctrl[30];
    uint32_t interrupts[4];
    uint32_t proc0InterruptEnables[4]; // TODO: proc1

    uint32_t inputs;

    ReadCallback readCallback;
};