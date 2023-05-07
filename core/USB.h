#pragma once

#include <cstdint>

#include "usbip.h"

class MemoryBus;

class USB final
{
public:
    USB(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(interruptEnables)
            update(target);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    void ramWrite(uint32_t addr);

    uint8_t *getRAM() {return dpram;}

    bool getEnabled();
    bool getConfigured();

    void startEnumeration();

    void setUSBIPEnabled(bool enabled);
    void usbipUpdate();

    bool usbipGetDescriptor(struct usbip_client *client, uint32_t seqnum, uint8_t descType, uint8_t descIndex, uint16_t setupIndex, uint16_t setupLength);
    bool usbipControlRequest(struct usbip_client *client, uint32_t seqnum, uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, uint16_t length, const uint8_t *outData);
    bool usbipIn(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length);
    bool usbipOut(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, const uint8_t *data);

private:
    void updateInterrupts();

    void updateEnumeration();

    void checkBuffer(int ep, bool in);

    MemoryBus &mem;

    uint64_t lastUpdate = 0;

    uint8_t dpram[4 * 1024];

    uint32_t mainCtrl;

    uint32_t sieCtrl;
    uint32_t sieStatus;

    uint32_t buffStatus; // todo: both buffers

    uint16_t bufferSelect;

    uint32_t interrupts;
    uint32_t interruptEnables;

    int enumerationState;
    uint8_t deviceDesc[18];
    uint8_t *configDesc;
    int configDescLen, configDescOffset;

    uint8_t cdcInEP, cdcOutEP;

    int cdcInOff;
    uint8_t cdcInData[1024];

    uint32_t shouldCheckBuffers;

    // usbip
    bool usbipEnabled = false;
    usbip_server *usbipServer = nullptr;
    usbip_device usbipDev;

    usbip_client *usbipLastClient;

    uint32_t usbipInSeqnum[16];
    uint32_t usbipOutSeqnum[16];

    uint8_t *usbipInData[16];
    uint32_t usbipInDataLen[16];
    uint32_t usbipInDataOffset[16];

    uint8_t *usbipOutData[16];
    uint32_t usbipOutDataLen[16];
    uint32_t usbipOutDataOffset[16];
};