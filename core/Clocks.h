#pragma once

#include <cstdint>

class Clocks final
{
public:
    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);
};