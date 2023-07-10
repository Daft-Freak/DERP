#pragma once

#include <cstdint>

#include "hardware/structs/i2c.h"

class MemoryBus;

class I2C final
{
public:
    I2C(MemoryBus &mem, int index);

    void reset();

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

private:
    MemoryBus &mem;
    int index;

    i2c_hw_t hw;
};