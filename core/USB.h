#pragma once

#include <cstdint>

#include "usbip.h"

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

    void setUSBIPEnabled(bool enabled);
    void usbipUpdate();

    bool usbipGetDescriptor(struct usbip_client *client, uint32_t seqnum, uint8_t descType, uint8_t descIndex, uint16_t setupIndex, uint16_t setupLength);
    bool usbipControlRequest(struct usbip_client *client, uint32_t seqnum, uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, uint16_t length, const uint8_t *outData);

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
    uint8_t deviceDesc[18];
    uint8_t *configDesc;
    int configDescLen, configDescOffset;

    uint8_t cdcInEP, cdcOutEP;

    int cdcInOff;
    uint8_t cdcInData[1024];

    // usbip
    bool usbipEnabled = false;
    usbip_server *usbipServer = nullptr;
    usbip_device usbipDev;

    usbip_client *usbipLastClient;
    uint32_t usbipControlSeqnum;
    bool usbipControlDir; // in == true

    uint8_t *usbipOutData = nullptr;
    uint32_t usbipOutDataLen = 0;
};