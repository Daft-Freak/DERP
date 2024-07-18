#include <cstdio>
#include <cstring>

#include "hardware/platform_defs.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/usb.h"
#include "hardware/regs/usb_device_dpram.h"

// need this for timeval on windows
// unfortunately, this results in windows.h getting pulled in
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include "winsock2.h"
#endif

#include "USB.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::USB;

static bool usbipGetDescriptor(struct usbip_client *client, uint32_t seqnum, uint8_t descType, uint8_t descIndex, uint16_t setupIndex, uint16_t setupLength, void *userData)
{
    auto usb = reinterpret_cast<USB*>(userData);
    return usb->usbipGetDescriptor(client, seqnum, descType, descIndex, setupIndex, setupLength);
}

static bool usbipControlRequest(struct usbip_client *client, uint32_t seqnum, uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, uint16_t length, const uint8_t *outData, void *userData)
{
    auto usb = reinterpret_cast<USB*>(userData);
    return usb->usbipControlRequest(client, seqnum, requestType, request, value, index, length, outData);
}

static bool usbipIn(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, void *userData)
{
    auto usb = reinterpret_cast<USB*>(userData);
    return usb->usbipIn(client, seqnum, ep, length);
}

static bool usbipOut(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, const uint8_t *data, void *userData)
{
    auto usb = reinterpret_cast<USB*>(userData);
    return usb->usbipOut(client, seqnum, ep, length, data);
}

static bool usbipUnlink(struct usbip_client *client, uint32_t seqnum, void *userData)
{
    auto usb = reinterpret_cast<USB*>(userData);
    return usb->usbipUnlink(client, seqnum);
}

USB::USB(MemoryBus &mem) : mem(mem)
{
}

void USB::reset()
{
    mainCtrl = USB_MAIN_CTRL_RESET;
    sieCtrl = USB_SIE_CTRL_RESET;
    sieStatus = USB_SIE_STATUS_RESET;
    buffStatus[0] = USB_BUFF_STATUS_RESET;
    buffStatus[1] = USB_BUFF_STATUS_RESET;
    bufferSelect = 0;
    epAbort = USB_EP_ABORT_RESET;
    epAbortDone = USB_EP_ABORT_DONE_RESET;

    interrupts = 0;
    interruptEnables = 0;

    enumerationState = EnumerationState::NotStarted;
    configDesc = nullptr;
    configDescLen = 0;
    configDescOffset = 0;

    cdcInEP = cdcOutEP = 0;
    cdcInOff = 0;

    shouldCheckBuffers = 0;

    if(usbipEnabled)
    {
        if(usbipServer)
        {
            usbip_destroy_server(usbipServer);
            usbipServer = nullptr;
        }

        if(usbip_create_server(&usbipServer, "::1", 0) != usbip_success)
            logf(LogLevel::Error, logComponent, "USBIP server create failed!");
    }

    for(auto &num : usbipInSeqnum)
        num = 0;
    for(auto &num : usbipOutSeqnum)
        num = 0;

    for(auto &buf : usbipInData)
        buf = nullptr;
    for(auto &len : usbipInDataLen)
        len = 0;
    for(auto &off : usbipInDataOffset)
        off = 0;

    for(auto &buf : usbipOutData)
        buf = nullptr;
    for(auto &len : usbipOutDataLen)
        len = 0;
    for(auto &off : usbipOutDataOffset)
        off = 0;
}

void USB::update(uint64_t target)
{
    if(shouldCheckBuffers)
    {
        usbipUpdate();

        for(int i = 0; i < 16; i++)
        {
            if(shouldCheckBuffers & (1 << i))
                checkBuffer(i, false);
            if(shouldCheckBuffers & (1 << (i + 16)))
                checkBuffer(i, true);
        }

        shouldCheckBuffers = 0;
    }

    lastUpdate = target;
}

uint64_t USB::getNextInterruptTime(uint64_t target)
{
    if(interruptEnables && shouldCheckBuffers)
        return lastUpdate + 1; // "now"

    return target;
}

uint32_t USB::regRead(uint32_t addr, uint64_t time)
{
    update(time);

    switch(addr)
    {
        case USB_MAIN_CTRL_OFFSET:
            return mainCtrl;

        case USB_SIE_CTRL_OFFSET:
            return sieCtrl;

        case USB_BUFF_STATUS_OFFSET:
            return buffStatus[0] | buffStatus[1];

        case USB_BUFF_CPU_SHOULD_HANDLE_OFFSET:
        {
            // if both are ready set based on the buffer selector, otherwise set if second buf
            auto bothMask = buffStatus[0] & buffStatus[1];
            return (bothMask & bufferSelect) | (~bothMask & buffStatus[1]);
        }

        case USB_EP_ABORT_OFFSET:
            return epAbort;
        case USB_EP_ABORT_DONE_OFFSET:
            return epAbortDone;

        case USB_INTS_OFFSET:
            return interrupts & interruptEnables; // TODO: force
    }

    logf(LogLevel::NotImplemented, logComponent, "R %04X", addr);

    return 0xBADADD55;
}

void USB::regWrite(uint32_t addr, uint32_t data, uint64_t time)
{
    update(time);

    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case USB_MAIN_CTRL_OFFSET:
            updateReg(mainCtrl, data, atomic);
            return;

        case USB_SIE_CTRL_OFFSET:
            updateReg(sieCtrl, data, atomic);
            return;

        case USB_SIE_STATUS_OFFSET:
            updateReg(sieStatus, data, atomic);
            if(enumerationState == EnumerationState::SetAddress && !(sieStatus & USB_SIE_STATUS_BUS_RESET_BITS))
                updateEnumeration();

            updateInterrupts();
            return;

        case USB_BUFF_STATUS_OFFSET:
            // these are marked as WC, but the code uses the clear alias...
            if(atomic == 3) // clear
            {
                auto bothMask = buffStatus[0] & buffStatus[1];
                auto shouldHandle = (bothMask & bufferSelect) | (~bothMask & buffStatus[1]);
                auto buf1Mask = data & shouldHandle;
                auto buf0Mask = data & ~buf1Mask;

                buffStatus[0] &= ~buf0Mask;
                buffStatus[1] &= ~buf1Mask;
            }
            else
                logf(LogLevel::NotImplemented, logComponent, "BUFF_STATUS%s%08X", op[atomic], data);

            if(atomic == 3 && (data & USB_BUFF_STATUS_EP0_IN_BITS)) // clearing ep0 in
            {
                if(enumerationState == EnumerationState::ReadDeviceDesc || enumerationState == EnumerationState::RequestConfigDesc || 
                   enumerationState == EnumerationState::RequestFullConfigDesc || enumerationState == EnumerationState::ReadConfigDesc || 
                   enumerationState == EnumerationState::InitCDC)
                {
                    updateEnumeration();
                }
            }

            updateInterrupts();
            return;

        case USB_EP_ABORT_OFFSET:
        {
            auto old = epAbort;
            if(updateReg(epAbort, data, atomic))
            {
                auto newSet = epAbort & ~old;
                auto availMask = (1 << 10 | 1 << 26);
                for(int ep = 0; ep < 16 && newSet; ep++, newSet >>= 2)
                {
                    if(newSet & 1)
                    {
                        auto bufCtrl = reinterpret_cast<uint32_t *>(dpram + 0x80 + ep * 8);

                        *bufCtrl &= ~availMask;
                        epAbortDone |= 1 << (ep * 2);
                    }
                    if(newSet & 2)
                    {
                        auto bufCtrl = reinterpret_cast<uint32_t *>(dpram + 0x80 + ep * 8 + 4);

                        *bufCtrl &= ~availMask;
                        epAbortDone |= 1 << (ep * 2 + 1);
                    }
                }
            }
            return;
        }
        case USB_EP_ABORT_DONE_OFFSET:
            updateReg(epAbortDone, data, atomic);
            return;

        case USB_INTE_OFFSET:
            updateReg(interruptEnables, data, atomic);
            return;
    }

    logf(LogLevel::NotImplemented, logComponent, "W %04X%s%08X", addr, op[atomic], data);
}

void USB::ramWrite(uint32_t addr)
{
    if(addr >= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_OFFSET && addr <= USB_DEVICE_DPRAM_EP15_OUT_BUFFER_CONTROL_OFFSET) // buffer control
    {
        bool in = !(addr & 4);
        int ep = (addr - 0x80) / 8;

        shouldCheckBuffers |= 1 << (ep + (in ? 16 : 0));
        mem.calcNextInterruptTime();
    }
}

bool USB::getEnabled()
{
    return (sieCtrl & USB_SIE_CTRL_PULLUP_EN_BITS); // TODO: check more things
}

bool USB::getConfigured()
{
    return false;
}

void USB::startEnumeration()
{
    if(enumerationState != EnumerationState::NotStarted)
        return;

    enumerationState = EnumerationState::RequestDeviceDesc;
    cdcInEP = cdcOutEP = 0;

    updateEnumeration();
}

void USB::updateInterrupts()
{
    const auto mask = USB_INTS_EP_STALL_NAK_BITS | USB_INTS_ABORT_DONE_BITS | USB_INTS_SETUP_REQ_BITS
                    | USB_INTS_BUS_RESET_BITS
                    | USB_INTS_VBUS_DETECT_BITS | USB_INTS_STALL_BITS | USB_INTS_ERROR_CRC_BITS | USB_INTS_ERROR_BIT_STUFF_BITS
                    | USB_INTS_ERROR_RX_OVERFLOW_BITS | USB_INTS_ERROR_RX_TIMEOUT_BITS | USB_INTS_ERROR_DATA_SEQ_BITS | USB_INTS_BUFF_STATUS_BITS
                    | USB_INTS_TRANS_COMPLETE_BITS;

    interrupts &= ~mask;

    // copy bits from SIE_STATUS
    if(sieStatus & USB_SIE_STATUS_VBUS_DETECTED_BITS)
        interrupts |= USB_INTS_VBUS_DETECT_BITS;
    if(sieStatus & USB_SIE_STATUS_SETUP_REC_BITS)
        interrupts |= USB_INTS_SETUP_REQ_BITS; 
    if(sieStatus & USB_SIE_STATUS_TRANS_COMPLETE_BITS)
        interrupts |= USB_INTS_TRANS_COMPLETE_BITS;
    if(sieStatus & USB_SIE_STATUS_BUS_RESET_BITS)
        interrupts |= USB_INTS_BUS_RESET_BITS;
    if(sieStatus & USB_SIE_STATUS_CRC_ERROR_BITS)
        interrupts |= USB_INTS_ERROR_CRC_BITS;
    if(sieStatus & USB_SIE_STATUS_BIT_STUFF_ERROR_BITS)
        interrupts |= USB_INTS_ERROR_BIT_STUFF_BITS;
    if(sieStatus & USB_SIE_STATUS_RX_OVERFLOW_BITS)
        interrupts |= USB_INTS_ERROR_RX_OVERFLOW_BITS;
    if(sieStatus & USB_SIE_STATUS_RX_TIMEOUT_BITS)
        interrupts |= USB_INTS_ERROR_RX_TIMEOUT_BITS;
    if(sieStatus & USB_SIE_STATUS_DATA_SEQ_ERROR_BITS)
        interrupts |= USB_INTS_ERROR_DATA_SEQ_BITS;
    if(sieStatus & USB_SIE_STATUS_STALL_REC_BITS)
        interrupts |= USB_INTS_STALL_BITS;

    //TODO: STALL_NAK

    if(epAbortDone)
        interrupts |= USB_INTS_ABORT_DONE_BITS;
    
    // set buff status
    if(buffStatus[0] | buffStatus[1])
        interrupts |= USB_INTS_BUFF_STATUS_BITS;

    if(interrupts & interruptEnables)
        mem.setPendingIRQ(USBCTRL_IRQ);
}

void USB::updateEnumeration()
{
    auto setupPacket = [this](uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, uint16_t length)
    {
        dpram[0] = requestType;
        dpram[1] = request;
        dpram[2] = value;
        dpram[3] = value >> 8; 
        dpram[4] = index;
        dpram[5] = index >> 8;
        dpram[6] = length;
        dpram[7] = length >> 8;

        sieStatus |= USB_SIE_STATUS_SETUP_REC_BITS;
    };

    switch(enumerationState)
    {
        case EnumerationState::NotStarted:
        case EnumerationState::Done:
            break;

        case EnumerationState::RequestDeviceDesc:
        {
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 1 << 8 /*device*/, 0, 18);

            enumerationState = EnumerationState::ReadDeviceDesc;
            break;
        }
        
        case EnumerationState::ReadDeviceDesc:
        {
            logf(LogLevel::Debug, logComponent, "USB device descriptor:");
            auto buf = dpram + 0x100;

            logf(LogLevel::Debug, logComponent, "\tbLength            %i", buf[0]);
            logf(LogLevel::Debug, logComponent, "\tbDescriptorType    %i", buf[1]);
            logf(LogLevel::Debug, logComponent, "\tbcdUSB             %i.%02i", buf[3], buf[2]);
            logf(LogLevel::Debug, logComponent, "\tbDeviceClass       %i", buf[4]);
            logf(LogLevel::Debug, logComponent, "\tbDeviceSubClass    %i", buf[5]);
            logf(LogLevel::Debug, logComponent, "\tbDeviceProtocol    %i", buf[6]);
            logf(LogLevel::Debug, logComponent, "\tbMaxPacketSize     %i", buf[7]);
            logf(LogLevel::Debug, logComponent, "\tidVendor           %04X", buf[8] | (buf[9] << 8));
            logf(LogLevel::Debug, logComponent, "\tidProduct          %04X", buf[10] | (buf[11] << 8));
            logf(LogLevel::Debug, logComponent, "\tbcdDevice          %i.%02i", buf[13], buf[12]);
            logf(LogLevel::Debug, logComponent, "\tiManufacturer      %i", buf[14]);
            logf(LogLevel::Debug, logComponent, "\tiProduct           %i", buf[15]);
            logf(LogLevel::Debug, logComponent, "\tiSerialNumber      %i", buf[16]);
            logf(LogLevel::Debug, logComponent, "\tbNumConfigurations %i", buf[17]);

            memcpy(deviceDesc, buf, 18);

            // reset
            sieStatus |= USB_SIE_STATUS_BUS_RESET_BITS;

            enumerationState = EnumerationState::SetAddress;
        }
        break;

        case EnumerationState::SetAddress: // reset done
        {
            setupPacket(0, 5 /*SET_ADDRESS*/, 1 /*addr*/, 0, 0);

            enumerationState = EnumerationState::RequestConfigDesc;
            break;
        }

        case EnumerationState::RequestConfigDesc: // set addr done
        {
            // get config descriptor
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 2 << 8 /*configuration*/, 0, 9);

            enumerationState = EnumerationState::RequestFullConfigDesc;
            break;
        }

        case EnumerationState::RequestFullConfigDesc:
        {
            auto buf = dpram + 0x100;
            configDescLen = buf[2] | buf[3] << 8;
            logf(LogLevel::Debug, logComponent, "USB config desc len %i", configDescLen);

            // now get the whole thing
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 2 << 8 /*configuration*/, 0, configDescLen);

            enumerationState = EnumerationState::ReadConfigDesc;

            if(configDesc)
                delete[] configDesc;

            configDesc = new uint8_t[configDescLen];
            configDescOffset = 0;
            break;
        }

        case EnumerationState::ReadConfigDesc:
        {
            auto buf = dpram + 0x100;
            auto bufLen = *reinterpret_cast<uint32_t *>(dpram + USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_OFFSET) & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LENGTH_0_BITS;
    
            memcpy(configDesc + configDescOffset, buf, bufLen);
            configDescOffset += bufLen;

            if(configDescOffset == configDescLen)
            {
                logf(LogLevel::Debug, logComponent, "USB configuration descriptor:");

                logf(LogLevel::Debug, logComponent, "\tbLength             %i", configDesc[0]);
                logf(LogLevel::Debug, logComponent, "\tbDescriptorType     %i", configDesc[1]);
                logf(LogLevel::Debug, logComponent, "\twTotalLength        %04X", configDesc[2] | configDesc[3] << 8);
                logf(LogLevel::Debug, logComponent, "\tbNumInterfaces      %i", configDesc[4]);
                logf(LogLevel::Debug, logComponent, "\tbConfigurationValue %i", configDesc[5]);
                logf(LogLevel::Debug, logComponent, "\tiConfiguration      %i", configDesc[6]);
                logf(LogLevel::Debug, logComponent, "\tbmAttributes        %02X", configDesc[7]);
                logf(LogLevel::Debug, logComponent, "\tbMaxPower           %i", configDesc[8]);

                // interfaces
                auto desc = configDesc + 9;
                auto end = desc + configDescOffset;
                for(int i = 0; i < configDesc[4] && desc < end;)
                {
                    if(!desc[0])
                        break;

                    if(desc[1] == 4)
                    {
                        logf(LogLevel::Debug, logComponent, "\tinterface descriptor:");
                        logf(LogLevel::Debug, logComponent, "\t\tbLength            %i", desc[0]);
                        logf(LogLevel::Debug, logComponent, "\t\tbDescriptorType    %i", desc[1]);
                        logf(LogLevel::Debug, logComponent, "\t\tbInterfaceNumber   %i", desc[2]);
                        logf(LogLevel::Debug, logComponent, "\t\tbAlternateSetting  %i", desc[3]);
                        logf(LogLevel::Debug, logComponent, "\t\tbNumEndpoints      %i", desc[4]);
                        logf(LogLevel::Debug, logComponent, "\t\tbInterfaceClass    %i", desc[5]);
                        logf(LogLevel::Debug, logComponent, "\t\tbInterfaceSubClass %i", desc[6]);
                        logf(LogLevel::Debug, logComponent, "\t\tbInterfaceProtocol %i", desc[7]);
                        logf(LogLevel::Debug, logComponent, "\t\tiInterface         %i", desc[8]);

                        int numEP = desc[4];

                        int intClass = desc[5];

                        i++;
                        desc += desc[0];
                        
                        // endpoints
                        for(int ep = 0; ep < numEP && desc < end;)
                        {
                            if(!desc[0])
                                break;

                            if(desc[1] == 5)
                            {
                                logf(LogLevel::Debug, logComponent, "\t\tendpoint descriptor:");
                                logf(LogLevel::Debug, logComponent, "\t\t\tbLength          %i", desc[0]);
                                logf(LogLevel::Debug, logComponent, "\t\t\tbDescriptorType  %i", desc[1]);
                                logf(LogLevel::Debug, logComponent, "\t\t\tbEndpointAddress %02X", desc[2]);
                                logf(LogLevel::Debug, logComponent, "\t\t\tbmAttributes     %i", desc[3]);
                                logf(LogLevel::Debug, logComponent, "\t\t\twMaxPacketSize   %04X", desc[4] | desc[5] << 8);
                                logf(LogLevel::Debug, logComponent, "\t\t\tbInterval        %i", desc[6]);
                                ep++;

                                if(intClass == 10 /*CDC Data*/)
                                {
                                    if(desc[2] & 0x80)
                                        cdcInEP = desc[2] & 0x7F;
                                    else
                                        cdcOutEP = desc[2];
                                }
                            }
                            else
                                logf(LogLevel::Debug, logComponent, "\t\tdesc %i len %i", desc[1], desc[0]);

                            desc += desc[0];
                        }
                    }
                    else
                    {
                        logf(LogLevel::Debug, logComponent, "\tdesc %i len %i", desc[1], desc[0]);
                        desc += desc[0];
                    }
                }

                if(usbipServer)
                {
                    // switch over to usbip now that we have the descriptors
                    usbip_remove_device(&usbipDev);

                    usbipDev.device_descriptor = deviceDesc;
                    usbipDev.config_descriptor = configDesc;
                    usbipDev.speed = usbip_speed_full;
                    usbipDev.user_data = this;
                    usbipDev.get_descriptor = ::usbipGetDescriptor;
                    usbipDev.control_request = ::usbipControlRequest;
                    usbipDev.in = ::usbipIn;
                    usbipDev.out = ::usbipOut;
                    usbipDev.unlink = ::usbipUnlink;

                    usbip_add_device(&usbipDev);

                    enumerationState = EnumerationState::Done;
                }
                else
                {
                    // set config
                    int configValue = configDesc[5];
                    setupPacket(0, 9 /*SET_CONFIGURATION*/, configValue, 0, 0);

                    enumerationState = EnumerationState::InitCDC;
                }
            }

            break;
        }

        case EnumerationState::InitCDC: // set config done
        {
            if(cdcInEP && cdcOutEP)
            {
                logf(LogLevel::Info, logComponent, "Found USB CDC IN %02X OUT %02X", cdcInEP, cdcOutEP);
                // setup the CDC interface if found
                setupPacket(0x21/*class, interface*/, 0x22 /*SET_CONTROL_LINE_STATE*/, 3, 0, 0);
            }

            enumerationState = EnumerationState::Done;
            
            break;
        }
    }

    updateInterrupts();
}

void USB::checkBuffer(int ep, bool in)
{
    auto bufCtrl = reinterpret_cast<uint32_t *>(dpram + 0x80 + ep * 8 + (in ? 0 : 4));
    auto ctrl = ep == 0 ? &sieCtrl : reinterpret_cast<uint32_t *>(dpram + ep * 8 + (in ? 0 : 4)); // ep0 interrupt/buffer bits in SIE_CTRL

    uint32_t epMask = 1 << (ep * 2 + !in);

    bool doubleBuffer = *ctrl & USB_SIE_CTRL_EP0_DOUBLE_BUF_BITS; 

    if(doubleBuffer)
    {
        if(*bufCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_RESET_BITS) // reset to buffer 0
            bufferSelect &= ~epMask;
    }

    // get the ctrl halfword for the current buffer
    int curBuffer = (bufferSelect & epMask) ? 1 : 0;
    uint32_t curBufferCtrl = *bufCtrl >> (curBuffer * 16);

    if(in)
    {
        // buffer available and full
        while(curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS && curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_0_BITS)
        {
            // "transfer" it
            uint32_t len = curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LENGTH_0_BITS;
            bool last = curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS;

            bool handled = true;

            // send usbip reply
            if(usbipServer && enumerationState == EnumerationState::Done)
            {
                if(usbipInSeqnum[ep])
                {
                    auto buf = ep ? dpram + (*ctrl & USB_DEVICE_DPRAM_EP1_IN_CONTROL_BUFFER_ADDRESS_BITS) : dpram + 0x100; // ep0 fixed buffer

                    if(curBuffer)
                        buf += 64;

                    // multiple packet transfer
                    if(usbipInData[ep])
                    {
                        if(usbipInDataLen[ep] - usbipInDataOffset[ep] < len)
                            len = usbipInDataLen[ep] - usbipInDataOffset[ep];
            
                        memcpy(usbipInData[ep] + usbipInDataOffset[ep], buf, len);
                        usbipInDataOffset[ep] += len;
                    }

                    // got all the data or not buffered
                    if(!usbipInData[ep] || usbipInDataOffset[ep] == usbipInDataLen[ep] || len < 64)
                    {
                        if(usbipInData[ep])
                            usbip_client_reply(usbipLastClient, usbipInSeqnum[ep], usbipInData[ep], usbipInDataOffset[ep]);
                        else
                            usbip_client_reply(usbipLastClient, usbipInSeqnum[ep], buf, len);

                        usbipInSeqnum[ep] = 0;
                    }
                }
                else if(len) // do nothing, wait for the client
                {
                    handled = false;
                }
            }
            else if(cdcInEP && ep == cdcInEP)
            {
                auto buf = dpram + (*ctrl & USB_DEVICE_DPRAM_EP1_IN_CONTROL_BUFFER_ADDRESS_BITS);

                while(len)
                {
                    int copyLen = std::min(len, static_cast<uint32_t>(sizeof(cdcInData) - cdcInOff));
                    memcpy(cdcInData + cdcInOff, buf, copyLen);
                    cdcInOff += copyLen;
                    len -= copyLen;

                    auto newLine = reinterpret_cast<uint8_t *>(memchr(cdcInData, '\n', cdcInOff));
                    while(newLine)
                    {
                        int off = newLine - cdcInData;
                        logf(LogLevel::Info, logComponent, "CDC: %.*s", off, cdcInData);

                        if(off != (cdcInOff - 1))
                            memmove(cdcInData, newLine + 1, cdcInOff - (off + 1));

                        cdcInOff -= (off + 1);
                        newLine = reinterpret_cast<uint8_t *>(memchr(cdcInData, '\n', cdcInOff));
                    }
                    
                    if(cdcInOff == sizeof(cdcInData))
                    {
                        cdcInOff = 0;
                        logf(LogLevel::Warning, logComponent, "Dropping CDC data");
                    }
                }
            }
            else if(ep)
                logf(LogLevel::Debug, logComponent, "EP%i in len %i last %i (bufCtrl %08X ctrl %08X)", ep, len, last, *bufCtrl, *ctrl);

            if(handled)
            {
                const auto mask = USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_PID_0_BITS | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LENGTH_0_BITS;

                // clear everything else, but only for this buffer
                if(curBuffer == 0)
                    *bufCtrl &= (0xFFFF0000 | mask);
                else
                    *bufCtrl &= (0xFFFF | mask << 16);

                if(last)
                    sieStatus |= USB_SIE_STATUS_TRANS_COMPLETE_BITS;

                // int for every buf or int on every two bufs and this is the second
                if(*ctrl & USB_SIE_CTRL_EP0_INT_1BUF_BITS || (curBuffer == 1 && *ctrl & USB_SIE_CTRL_EP0_INT_2BUF_BITS))
                    buffStatus[curBuffer] |= epMask; // EPx_IN
                
                updateInterrupts();

                // swap buffers
                if(doubleBuffer)
                {
                    bufferSelect ^= epMask;
                    curBuffer ^= 1;
                }

                // re-read status (possibly for the other buffer)
                curBufferCtrl = *bufCtrl >> (curBuffer * 16);
            }
            else
                break;
        }
    }
    else
    {
        // buffer available and not full
        while(curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_AVAILABLE_0_BITS && !(curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_0_BITS))
        {
            uint32_t len = curBufferCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LENGTH_0_BITS;

            bool haveData = false;
            
            // copy usbip data and send reply
            if(usbipOutSeqnum[ep])
            {
                if(usbipOutData[ep])
                {
                    if(usbipOutDataLen[ep] - usbipOutDataOffset[ep] < len)
                        len = usbipOutDataLen[ep] - usbipOutDataOffset[ep];

                    auto buf = ep ? dpram + (*ctrl & USB_DEVICE_DPRAM_EP1_IN_CONTROL_BUFFER_ADDRESS_BITS) : dpram + 0x100; // ep0 fixed buffer
                    if(curBuffer)
                        buf += 64;

                    memcpy(buf, usbipOutData[ep] + usbipOutDataOffset[ep], len);
                    usbipOutDataOffset[ep] += len;

                    if(usbipOutDataOffset[ep] == usbipOutDataLen[ep])
                    {
                        delete[] usbipOutData[ep];
                        usbipOutData[ep] = nullptr;
                    }

                    haveData = true;
                }

                if(usbipOutDataOffset[ep] == usbipOutDataLen[ep])
                {
                    usbip_client_reply(usbipLastClient, usbipOutSeqnum[ep], nullptr, usbipOutDataOffset[ep]);

                    usbipOutSeqnum[ep] = 0;
                }
            }

            if(len == 0 || haveData)
            {
                bool last = *bufCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS;

                const auto mask = USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_LAST_0_BITS | USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_PID_0_BITS;

                // clear everything else, but only for this buffer
                if(curBuffer == 0)
                {
                    *bufCtrl &= (0xFFFF0000 | mask);
                    *bufCtrl |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_0_BITS | len;
                }
                else
                {
                    *bufCtrl &= (0xFFFF | mask << 16);
                    *bufCtrl |= USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_FULL_1_BITS | len << 16;
                }

                if(last)
                    sieStatus |= USB_SIE_STATUS_TRANS_COMPLETE_BITS;
                    
                // int for every buf or int on every two bufs and this is the second
                if(*ctrl & (1 << 29) || (curBuffer == 1 && *ctrl & (1 << 30)))
                    buffStatus[curBuffer] |= epMask; // EPx_OUT

                updateInterrupts();

                // swap buffers
                if(doubleBuffer)
                {
                    bufferSelect ^= epMask;
                    curBuffer ^= 1;
                }

                // re-read status (possibly for the other buffer)
                curBufferCtrl = *bufCtrl >> (curBuffer * 16);
            }
            else
                break;
        }
    }

    if(*bufCtrl & USB_DEVICE_DPRAM_EP0_IN_BUFFER_CONTROL_STALL_BITS)
    {
        // send usbip stall
        // TODO EP_STALL_ARM for EP0

        auto &seqnum = in ? usbipInSeqnum[ep] : usbipOutSeqnum[ep];
        if(seqnum)
        {
            usbip_client_stall(usbipLastClient, seqnum);

            seqnum = 0;
        }
    }
}

void USB::setUSBIPEnabled(bool enabled)
{
    usbipEnabled = enabled;
}

void USB::usbipUpdate()
{
    if(!usbipServer)
        return;

    timeval timeout;

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    usbip_server_update(usbipServer, &timeout);
}

bool USB::usbipGetDescriptor(struct usbip_client *client, uint32_t seqnum, uint8_t descType, uint8_t descIndex, uint16_t setupIndex, uint16_t setupLength)
{
    // re-assemble setup packet
    dpram[0] = 0x80;
    dpram[1] = 0x6;
    dpram[2] = descIndex;
    dpram[3] = descType; 
    dpram[4] = setupIndex;
    dpram[5] = setupIndex >> 8;
    dpram[6] = setupLength;
    dpram[7] = setupLength >> 8;

    sieStatus |= USB_SIE_STATUS_SETUP_REC_BITS;

    usbipLastClient = client;
    usbipInSeqnum[0] = seqnum;

    updateInterrupts();

    return true; // will send stall later if needed
}

bool USB::usbipControlRequest(struct usbip_client *client, uint32_t seqnum, uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, uint16_t length, const uint8_t *outData)
{
    if(outData) // copy out data
    {
        if(usbipOutData[0])
            delete[] usbipOutData[0];

        usbipOutData[0] = new uint8_t[length];
        usbipOutDataLen[0] = length;
        usbipOutDataOffset[0] = 0;
        memcpy(usbipOutData[0], outData, length);
    }

    // re-assemble setup packet
    dpram[0] = requestType;
    dpram[1] = request;
    dpram[2] = value;
    dpram[3] = value >> 8; 
    dpram[4] = index;
    dpram[5] = index >> 8;
    dpram[6] = length;
    dpram[7] = length >> 8;

    sieStatus |= USB_SIE_STATUS_SETUP_REC_BITS;

    bool in = (requestType & 0x80) || !length; // 0-length always in?

    usbipLastClient = client;

    if(in)
        usbipInSeqnum[0] = seqnum;
    else
        usbipOutSeqnum[0] = seqnum;

    updateInterrupts();

    return true; // will send stall later if needed
}

bool USB::usbipIn(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length)
{
    if(usbipInData[ep])
        delete[] usbipInData[ep];

    usbipInData[ep] = new uint8_t[length];

    usbipInDataLen[ep] = length;
    usbipInDataOffset[ep] = 0;

    usbipInSeqnum[ep] = seqnum;
    usbipLastClient = client;

    shouldCheckBuffers |= (1 << (ep + 16));
    mem.calcNextInterruptTime();

    return true;
}

bool USB::usbipOut(struct usbip_client *client, uint32_t seqnum, int ep, uint32_t length, const uint8_t *data)
{
    if(usbipOutData[ep])
        delete[] usbipOutData[ep];

    usbipOutData[ep] = new uint8_t[length];
    usbipOutDataLen[ep] = length;
    usbipOutDataOffset[ep] = 0;
    memcpy(usbipOutData[ep], data, length);

    usbipOutSeqnum[ep] = seqnum;
    usbipLastClient = client;

    shouldCheckBuffers |= (1 << ep);
    mem.calcNextInterruptTime();

    return true;
}

bool USB::usbipUnlink(struct usbip_client *client, uint32_t seqnum)
{
    for(auto &n : usbipInSeqnum)
    {
        if(n == seqnum)
        {
            n = 0;
            return true;
        }
    }

    for(auto &n : usbipOutSeqnum)
    {
        if(n == seqnum)
        {
            n = 0;
            return true;
        }
    }

    return true; // should be more strict here, but we're losing seqnums sometimes
}