#pragma once

#include <cstdint>

class GPIO final
{
public:
    void reset();

    // IO_BANK0
    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    // PADS_BANK0
    uint32_t padsRegRead(uint32_t addr);
    void padsRegWrite(uint32_t addr, uint32_t data);

private:
    uint32_t ctrl[30];
    uint32_t interrupts[4];
    uint32_t proc0InterruptEnables[4]; // TODO: proc1
};