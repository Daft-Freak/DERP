#pragma once

#include <cstdint>
#include <functional>

#include "hardware/structs/i2c.h"

class MemoryBus;

class I2C final
{
public:
    using WriteCallback = std::function<void(uint64_t, I2C &, uint8_t, bool)>;

    I2C(MemoryBus &mem, int index);

    void reset();

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    void setWriteCallback(WriteCallback cb);

    uint16_t getTargetAddr() const;

private:
    MemoryBus &mem;
    int index;

    i2c_hw_t hw;

    WriteCallback writeCallback;
};