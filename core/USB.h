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

    void ramWrite(uint32_t addr);

    uint8_t *getRAM() {return dpram;}

    bool getEnabled();
    bool getConfigured();

    void startEnumeration();

private:
    void updateInterrupts();

    void updateEnumeration();

    MemoryBus &mem;

    uint8_t dpram[4 * 1024];

    uint32_t mainCtrl;

    uint32_t sieCtrl;
    uint32_t sieStatus;

    uint32_t buffStatus;

    uint32_t interrupts;
    uint32_t interruptEnables;

    int enumerationState;
    uint8_t *configDesc;
    int configDescLen, configDescOffset;
};