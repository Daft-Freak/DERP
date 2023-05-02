#pragma once

#include <cstdint>

class MemoryBus;

class USB final
{
public:
    USB(MemoryBus &mem);

    void reset();

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    uint8_t *getRAM() {return dpram;}

private:
    MemoryBus &mem;

    uint8_t dpram[4 * 1024];
};