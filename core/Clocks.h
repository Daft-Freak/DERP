#pragma once

#include <cstdint>

class Clocks final
{
public:
    void reset();

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

private:

    uint32_t ctrl[10];
    uint32_t div[10];
};