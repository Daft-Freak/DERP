#include <SDL.h>

#include "PicoVision.h"

#include "Logging.h"
#include "MemoryBus.h"

#include "hardware/regs/intctrl.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

extern void updateScreenSettings(); // TODO: declare this... somewhere

static const int modeWidth[]
{
    640,
    720,
    720,
    720,

    // "wide"
    800,
    800,
    800,
    960,
    960,
    1280,
};

static const int modeHeight[]
{
    480,
    480,
    400,
    576,

    // "wide"
    600,
    480,
    450,
    540,
    540,
    720,
};

PicoVisionBoard::PicoVisionBoard(MemoryBus &mem) : mem(mem)
{
    mem.setInterruptUpdateCallback([this](auto time, auto irqMask){onInterruptUpdate(time, irqMask);});
    mem.setGetNextInterruptTimeCallback([this](auto time){return onGetNextInterruptTime(time);});

    mem.getGPIO().setReadCallback([this](auto time, auto &gpio){onGPIORead(time, gpio);});
    mem.getGPIO().clearInputFloatingMask(1 << 16); // vsync

    displayClock.setFrequency(60 * 1000);
    mem.getClocks().addClockTarget(-1, displayClock);

    mem.getPIO(1).setTXCallback([this](auto time, auto &pio, auto sm, auto data){onPIOTX(time, pio, sm, data);});

    mem.getI2C(1).setWriteCallback([this](auto time, auto &i2c, auto data, auto stop){onI2CWrite(time, i2c, data, stop);});
}

void PicoVisionBoard::getScreenSize(int &w, int &h)
{
    int mode = i2cRegData[0xFC];

    if(mode >= 0x10)
        mode -= 12;

    w = modeWidth[mode];
    h = modeHeight[mode];
}

int PicoVisionBoard::getScreenFormat()
{
    // TODO: also configurable
    // but set per-line, so using the highest bit depth is the easiest thing to do...
    return SDL_PIXELFORMAT_BGR24;
}

const uint8_t *PicoVisionBoard::getScreenData()
{
    return screenData;
}

void PicoVisionBoard::update(uint64_t time)
{
    displayUpdate(time);
}

void PicoVisionBoard::displayUpdate(uint64_t time, bool forIntr)
{
    auto ticks = displayClock.getCyclesToTime(time);

    if(!ticks)
        return;

    // do nothing if not enabled yet
    if(!i2cRegData[0xFD])
    {
        displayClock.addCycles(ticks);
        return;
    }

    while(ticks)
    {
        // tick 60 * 1000 times a second to create a short vsync pulse
        const uint32_t numDisplayTicks = 1000;

        int step = std::max(1u, std::min(ticks, numDisplayTicks - 1 - displayTick));
        ticks -= step;

        displayClock.addCycles(step);

        // set/clear vsync
        auto newTick = (displayTick + step) % numDisplayTicks;

        if(newTick == numDisplayTicks - 1) // now last tick
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().setInputMask(1 << 16);

            mem.getPIO(1).update(displayClock.getTime()); // bit of a hack to workaround sync issues
            updateScreenData();
        }
        else if(displayTick == numDisplayTicks - 1) // was last tick
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().clearInputMask(1 << 16);
        }

        displayTick = newTick;
    }
}

void PicoVisionBoard::updateScreenData()
{
    if(!i2cRegData[0xFD])
        return;

    int ramBank = (mem.getGPIO().getPadState() & (1 << 8)) ? 0 : 1;
    auto psram = psramData[ramBank];

    // DVI setup
    if(psram[0] != 'P' || psram[1] != 'I' || psram[2] != 'C' || psram[3] != 'O')
        return;

    // TODO
    //auto resolutionSelect = psram[4];
    auto vRepeat = psram[6];
    auto enabled = psram[7];

    // TODO
    //uint16_t hOff = psram[8] | psram[9] << 8;
    uint16_t hLen = psram[10] | psram[11] << 8;
    // TODO
    /*
    uint16_t vOff = psram[12] | psram[13] << 8;
    uint16_t vLen = psram[14] | psram[15] << 8;
    */

    if(!enabled)
        return;

    // frame table header
    // TODO: multiple frames
    uint16_t frameTableLen = psram[20] | psram[21] << 8;
    // TODO: palettes, sprites

    int offset = 28; // + index * frameTableLen * 4

    for(unsigned line = 0; line < frameTableLen; line++) 
    {
        auto addr = psram[offset] | psram[offset + 1] << 8 | psram[offset + 2] << 16;
        offset += 3;

        auto meta = psram[offset++];

        auto scrollIndex = meta >> 5;
        auto format = (meta >> 3) & 3;
        auto hRepeat = meta & 0x7;

        // apply scroll offset
        int32_t offset = 0;
        if(scrollIndex)
        {
            auto scrollData = scrollGroupData + (scrollIndex - 1) * 13;

            offset = scrollData[0] | scrollData[1] << 8 | scrollData[2] << 16;

            // also max addr, offset2, wrap position and wrap offset (for wrapping)
        }

        addr += offset;

        static const int formatBytes[]{2, 2, 1, 3};

        for(int x = 0; x < hLen; x += hRepeat)
        {
            for(int r = 0; r < hRepeat; r++)
            {
                auto outOff = (x + r + line * vRepeat * hLen) * 3;
                if(format == 1) // RGB555
                {
                    uint16_t rgb555 = psram[addr] | psram[addr + 1] << 8;
                    auto b =  rgb555        & 0x1F;
                    auto g = (rgb555 >>  5) & 0x1F;
                    auto r = (rgb555 >> 10) & 0x1F;

                    r = r << 3 | r >> 2;
                    g = g << 3 | g >> 2;
                    b = b << 3 | b >> 2;

                    screenData[outOff + 0] = b;
                    screenData[outOff + 1] = g;
                    screenData[outOff + 2] = r;
                }
                // TODO: paletted
                else if(format == 3) // RGB8
                {
                    screenData[outOff + 0] = psram[addr + 0];
                    screenData[outOff + 1] = psram[addr + 1];
                    screenData[outOff + 2] = psram[addr + 2];
                }
            }
            addr += formatBytes[format];

            // v repeat
            for(int r = 1; r < vRepeat; r++)
                memcpy(&screenData[(line * vRepeat + r) * hLen * 3], &screenData[line * vRepeat * hLen * 3], hLen * 3);
        }
    }
}

void PicoVisionBoard::onGPIORead(uint64_t time, GPIO &gpio)
{
    displayUpdate(time);
}

void PicoVisionBoard::onInterruptUpdate(uint64_t time, uint32_t irqMask)
{
    if(i2cRegData[0xFD] && (irqMask & (1 << IO_IRQ_BANK0)))
        displayUpdate(time, true);
}

uint64_t PicoVisionBoard::onGetNextInterruptTime(uint64_t time)
{
    if(!i2cRegData[0xFD] || !mem.getGPIO().interruptsEnabledOnPin(16))
        return time;

    int lines = std::max(1, 999 - displayTick);
    auto ret = displayClock.getTimeToCycles(lines);

    return ret;
}

void PicoVisionBoard::onPIOTX(uint64_t time, PIO &pio, int sm, uint32_t data)
{
    // 32blit-sdk SD card
    if(sm == 1)
        pio.getRXFIFO(1).push(0xFFFFFFFF);

    if(sm != 0)
        return;

    // PSRAM
    int ramBank = (mem.getGPIO().getPadState() & (1 << 8)) ? 1 : 0;

    auto cmdLenWords = ramCmdLenWords[ramBank];
    auto cmdOffset = ramCmdOffset[ramBank];
    auto dataLenBytes = ramDataLenBytes[ramBank];

    bool isResetProg = !ramQuadEnabled[ramBank]; // assume reset program until quad enabled

    if(cmdOffset == cmdLenWords)
    {
        cmdOffset = 0;

        if(isResetProg)
            ramCmdLenWords[ramBank] = cmdLenWords = data / 32 + 2;
        else
            ramCmdLenWords[ramBank] = cmdLenWords = (data + 1) * (ramQuadEnabled[ramBank] ? 4 : 1) / 32 + 4;

        ramDataLenBytes[ramBank] = dataLenBytes = (data + 1) * (ramQuadEnabled[ramBank] ? 4 : 1) / 8;
    }
    else if(cmdOffset == 1)
    {
        auto cmd = ramCommand[ramBank] = data >> 24;

        if(cmd == 0x35)
        {
            logf(LogLevel::Debug, logComponent, "psram %i quad enabled", ramBank);
            ramQuadEnabled[ramBank] = true;
        }
        else if(cmd == 0x38) // write
            ramCmdAddr[ramBank] = data & 0xFFFFFF;
        else if(cmd == 0xEB) // read
        {
            ramCmdAddr[ramBank] = data & 0xFFFFFF;
            ramCmdLenWords[ramBank] = 3; // the "data" part is read, not written
        }
        else
            logf(LogLevel::Debug, logComponent, "psram %i cmd %02X %06X len %i", ramBank, cmd, data & 0xFFFFFF, dataLenBytes);
    }
    else if(cmdOffset == 2 && !isResetProg)
    {
        // pio addr
    }
    else // data
    {
        if(ramCommand[ramBank] == 0x38) // write
        {
            auto offset = (cmdOffset - 3) * 4;

            int len = std::min(4, dataLenBytes - offset);

            auto ptr = psramData[ramBank] + ramCmdAddr[ramBank] + offset;

            for(int i = 0; i < len; i++)
            {
                *ptr++ = data >> 24;

                data <<= 8;
            }
        }
    }

    cmdOffset++;

    ramCmdOffset[ramBank] = cmdOffset;
}

void PicoVisionBoard::onI2CWrite(uint64_t time, I2C &i2c, uint8_t data, bool stop)
{
    if(i2c.getTargetAddr() != 0x0D)
        return;

    if(i2cReg == -1)
    {
        i2cReg = data;
        i2cRegIndex = 0;
    }
    else
    {
        logf(LogLevel::Info, logComponent, "I2C reg %02X = %02X", i2cReg, data);

        // update screen settings on enable
        if(i2cReg == 0xFD && data && !i2cRegData[i2cReg])
            updateScreenSettings();

        // scroll registers are E1-E8
        if(i2cReg >= 0xE1 && i2cReg < 0xE8)
        {
            int offset = (i2cReg - 0xE1) * 13 + i2cRegIndex;

            scrollGroupData[offset] = data;

            if(i2cRegIndex++ == 13)
                i2cRegIndex = 0;
        }
        else
            i2cRegData[i2cReg++] = data;
    }

    // TODO: need to keep for reads
    if(stop)
        i2cReg = -1;
}