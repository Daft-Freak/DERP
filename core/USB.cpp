#include <cstdio>
#include <cstring>

#include "USB.h"

#include "MemoryBus.h"

USB::USB(MemoryBus &mem) : mem(mem)
{
}

void USB::reset()
{
    mainCtrl = 0;
    sieCtrl = 0;
    sieStatus = 0;
    buffStatus = 0;

    interrupts = 0;
    interruptEnables = 0;

    enumerationState = 0;
    configDesc = nullptr;
    configDescLen = 0;
    configDescOffset = 0;

    cdcInEP = cdcOutEP = 0;
    cdcInOff = 0;
}

uint32_t USB::regRead(uint32_t addr)
{
    switch(addr)
    {
        case 0x40: // MAIN_CTRL
            return mainCtrl;

        case 0x58: // BUFF_STATUS
            return buffStatus;

        case 0x98: // INTS
            return interrupts & interruptEnables; // TODO: force
    }

    printf("USB R %04X\n", addr);

    return 0xBADADD55;
}

void USB::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
        case 0x40: // MAIN_CTRL
            updateReg(mainCtrl, data, atomic);
            return;

        case 0x4C: // SIE_CTRL
            updateReg(sieCtrl, data, atomic);
            return;

        case 0x50: // SIE_STATUS
            updateReg(sieStatus, data, atomic);
            if(enumerationState == 3 && !(sieStatus & (1 << 19)/*BUS_RESET*/))
                updateEnumeration();

            updateInterrupts();
            return;

        case 0x58: // BUFF_STATUS
            updateReg(buffStatus, data, atomic);

            if(atomic == 3 && (data & 1)) // clearing ep0 in
            {
                if(enumerationState == 2 || enumerationState == 4 || enumerationState == 5 || enumerationState == 6 || enumerationState == 7)
                    updateEnumeration();
            }

            updateInterrupts();
            return;

        case 0x90: // INTE
            updateReg(interruptEnables, data, atomic);
            return;
    }

    printf("USB W %04X%s%08X\n", addr, op[atomic], data);
}

void USB::ramWrite(uint32_t addr)
{
    if(addr >= 0x80 && addr < 0x100) // buffer control
    {
        bool in = !(addr & 4);
        int ep = (addr - 0x80) / 8;

        auto bufCtrl = reinterpret_cast<uint32_t *>(dpram + addr);
        auto ctrl = ep == 0 ? &sieCtrl : reinterpret_cast<uint32_t *>(dpram + ep * 8 + (in ? 0 : 4)); // ep0 interrupt/buffer bits in SIE_CTRL

        if(*ctrl & (1 << 30))
            printf("ep%i db\n", ep);

        if(in)
        {
            if(*bufCtrl & (1 << 10) && *bufCtrl & (1 << 15)) // buf 0 available and full
            {
                // "transfer" it
                int len = *bufCtrl & 0x3FF;
                bool last = *bufCtrl & (1 << 14);

                if(cdcInEP && ep == cdcInEP)
                {
                    auto buf = dpram + (*ctrl & 0xFFF);

                    while(len)
                    {
                        int copyLen = std::min(len, static_cast<int>(sizeof(cdcInData) - cdcInOff));
                        memcpy(cdcInData + cdcInOff, buf, copyLen);
                        cdcInOff += copyLen;
                        len -= copyLen;

                        auto newLine = reinterpret_cast<uint8_t *>(memchr(cdcInData, '\n', cdcInOff));
                        while(newLine)
                        {
                            int off = newLine - cdcInData;
                            printf("USBCDC: %.*s", off + 1, cdcInData);

                            if(off != (cdcInOff - 1))
                                memmove(cdcInData, newLine + 1, cdcInOff - (off + 1));

                            cdcInOff -= (off + 1);
                            newLine = reinterpret_cast<uint8_t *>(memchr(cdcInData, '\n', cdcInOff));
                        }
                        
                        if(cdcInOff == sizeof(cdcInData))
                        {
                            cdcInOff = 0;
                            printf("Dropping CDC data\n");
                        }
                    }
                }
                else if(ep)
                    printf("EP%i in len %i last %i (bufCtrl %08X ctrl %08X)\n", ep, len, last, *bufCtrl, *ctrl);

                *bufCtrl &= 0x63FF;

                if(last)
                {
                    sieStatus |= (1 << 18); /*TRANS_COMPLETE*/
                    
                    if(*ctrl & (1 << 29))
                        buffStatus |= 1 << (ep * 2); // EPx_IN
                    
                    updateInterrupts();
                }
            }
        }
        else
        {
            if(*bufCtrl & (1 << 10) && !(*bufCtrl & (1 << 15))) // buf 0 available and not full
            {
                int len = *bufCtrl & 0x3FF;

                if(len)
                    printf("EP%i out len %i\n", ep,  len);
                else
                {
                    bool last = *bufCtrl & (1 << 14);
                    *bufCtrl &= 0x63FF;
                    *bufCtrl |= (1 << 15); // full

                    if(last)
                    {
                        sieStatus |= (1 << 18); /*TRANS_COMPLETE*/
                        
                        if(*ctrl & (1 << 29))
                            buffStatus |= 1 << (ep * 2 + 1); // EPx_OUT
                        
                        updateInterrupts();
                    }
                }
            }
        }
    }

}

bool USB::getEnabled()
{
    return (sieCtrl & (1 << 16)/*PULLUP_EN*/); // TODO: check more things
}

bool USB::getConfigured()
{
    return false;
}

void USB::startEnumeration()
{
    if(enumerationState)
        return;

    enumerationState = 1;
    cdcInEP = cdcOutEP = 0;

    updateEnumeration();
}

void USB::updateInterrupts()
{
    interrupts &= ~0xD1FF8;

    // copy bits from SIE_STATUS
    if(sieStatus & (1 << 17)) // SETUP_REQ
        interrupts |= (1 << 16);
    if(sieStatus & (1 << 18)) // TRANS_COMPLETE
        interrupts |= (1 << 3);
    if(sieStatus & (1 << 19)) // BUS_RESET
        interrupts |= (1 << 12);

    // set buff status
    if(buffStatus)
        interrupts |= (1 << 4);

    if(interrupts & interruptEnables)
        mem.setPendingIRQ(5/*USBCTRL_IRQ*/);
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

        sieStatus |= (1 << 17)/*SETUP_REQ*/;
    };

    switch(enumerationState)
    {
        case 1: // request device descriptor
        {
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 1 << 8 /*device*/, 0, 18);

            enumerationState = 2;
            break;
        }
        
        case 2: // read device descriptor
        {
            printf("USB device descriptor:\n");
            auto buf = dpram + 0x100;

            printf("\tbLength            %i\n", buf[0]);
            printf("\tbDescriptorType    %i\n", buf[1]);
            printf("\tbcdUSB             %i.%02i\n", buf[3], buf[2]);
            printf("\tbDeviceClass       %i\n", buf[4]);
            printf("\tbDeviceSubClass    %i\n", buf[5]);
            printf("\tbDeviceProtocol    %i\n", buf[6]);
            printf("\tbMaxPacketSize     %i\n", buf[7]);
            printf("\tidVendor           %04X\n", buf[8] | (buf[9] << 8));
            printf("\tidProduct          %04X\n", buf[10] | (buf[11] << 8));
            printf("\tbcdDevice          %i.%02i\n", buf[13], buf[12]);
            printf("\tiManufacturer      %i\n", buf[14]);
            printf("\tiProduct           %i\n", buf[15]);
            printf("\tiSerialNumber      %i\n", buf[16]);
            printf("\tbNumConfigurations %i\n", buf[17]);

            // reset
            sieStatus |= (1 << 19); // BUS_RESET

            enumerationState = 3;
        }
        break;

        case 3: // reset done
        {
            setupPacket(0, 5 /*SET_ADDRESS*/, 1 /*addr*/, 0, 0);

            enumerationState = 4;
            break;
        }

        case 4: // set addr done
        {
            // get config descriptor
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 2 << 8 /*configuration*/, 0, 9);

            enumerationState = 5;
            break;
        }

        case 5: // read config descriptor
        {
            auto buf = dpram + 0x100;
            configDescLen = buf[2] | buf[3] << 8;
            printf("USB config desc len %i\n", configDescLen);

            // now get the whole thing
            setupPacket(0x80, 6 /*GET_DESCRIPTOR*/, 2 << 8 /*configuration*/, 0, configDescLen);

            enumerationState = 6;

            if(configDesc)
                delete[] configDesc;

            configDesc = new uint8_t[configDescLen];
            configDescOffset = 0;
            break;
        }

        case 6: // REALLY read config descriptor
        {
            auto buf = dpram + 0x100;
            auto bufLen = *reinterpret_cast<uint32_t *>(dpram + 0x80) & 0x3FF;
    
            memcpy(configDesc + configDescOffset, buf, bufLen);
            configDescOffset += bufLen;

            if(configDescOffset == configDescLen)
            {
                printf("USB configuration descriptor:\n");

                printf("\tbLength             %i\n", configDesc[0]);
                printf("\tbDescriptorType     %i\n", configDesc[1]);
                printf("\twTotalLength        %04X\n", configDesc[2] | configDesc[3] << 8);
                printf("\tbNumInterfaces      %i\n", configDesc[4]);
                printf("\tbConfigurationValue %i\n", configDesc[5]);
                printf("\tiConfiguration      %i\n", configDesc[6]);
                printf("\tbmAttributes        %02X\n", configDesc[7]);
                printf("\tbMaxPower           %i\n", configDesc[8]);

                // interfaces
                auto desc = configDesc + 9;
                auto end = desc + configDescOffset;
                for(int i = 0; i < configDesc[4] && desc < end;)
                {
                    if(!desc[0])
                        break;

                    if(desc[1] == 4)
                    {
                        printf("\tinterface descriptor:\n");
                        printf("\t\tbLength            %i\n", desc[0]);
                        printf("\t\tbDescriptorType    %i\n", desc[1]);
                        printf("\t\tbInterfaceNumber   %i\n", desc[2]);
                        printf("\t\tbAlternateSetting  %i\n", desc[3]);
                        printf("\t\tbNumEndpoints      %i\n", desc[4]);
                        printf("\t\tbInterfaceClass    %i\n", desc[5]);
                        printf("\t\tbInterfaceSubClass %i\n", desc[6]);
                        printf("\t\tbInterfaceProtocol %i\n", desc[7]);
                        printf("\t\tiInterface         %i\n", desc[8]);

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
                                printf("\t\tendpoint descriptor:\n");
                                printf("\t\t\tbLength          %i\n", desc[0]);
                                printf("\t\t\tbDescriptorType  %i\n", desc[1]);
                                printf("\t\t\tbEndpointAddress %02X\n", desc[2]);
                                printf("\t\t\tbmAttributes     %i\n", desc[3]);
                                printf("\t\t\twMaxPacketSize   %04X\n", desc[4] | desc[5] << 8);
                                printf("\t\t\tbInterval        %i\n", desc[6]);
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
                                printf("\t\tdesc %i len %i\n", desc[1], desc[0]);

                            desc += desc[0];
                        }
                    }
                    else
                    {
                        printf("\tdesc %i len %i\n", desc[1], desc[0]);
                        desc += desc[0];
                    }
                }

                // set config
                int configValue = configDesc[5];
                setupPacket(0, 9 /*SET_CONFIGURATION*/, configValue, 0, 0);

                enumerationState = 7;
            }

            break;
        }

        case 7: // set config done
        {
            if(cdcInEP && cdcOutEP)
            {
                printf("Found USB CDC IN %02X OUT %02X\n", cdcInEP, cdcOutEP);
                // setup the CDC interface if found
                setupPacket(0x21/*class, interface*/, 0x22 /*SET_CONTROL_LINE_STATE*/, 3, 0, 0);
            }

            enumerationState = 8;
            
            break;
        }
    }

    updateInterrupts();
}