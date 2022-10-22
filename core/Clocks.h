#pragma once

#include <cstdint>

class Clocks final
{
public:
    void reset();

    void update(int ms);

    uint32_t getClockVal(int clock);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    // also handle the PLLs here
    uint32_t pllSysRegRead(uint32_t addr);
    void pllSysRegWrite(uint32_t addr, uint32_t data);

    uint32_t pllUSBRegRead(uint32_t addr);
    void pllUSBRegWrite(uint32_t addr, uint32_t data);

private:
    void calcFreq(int clock);

    uint32_t ctrl[10];
    uint32_t div[10];

    uint32_t clockVal[10];
    uint32_t clockFreq[10];

    uint32_t pllSysCS, pllUSBCS;
    uint32_t pllSysPWR, pllUSBPWR;
    uint32_t pllSysFBDIV, pllUSBFBDIV;
    uint32_t pllSysPRIM, pllUSBPRIM;
};