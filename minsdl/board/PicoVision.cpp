#include <SDL.h>

#include "PicoVision.h"

#include "Logging.h"
#include "MemoryBus.h"

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
    mem.getGPIO().setReadCallback([this](auto time, auto &gpio){onGPIORead(time, gpio);});
    mem.getGPIO().clearInputFloatingMask(1 << 16); // vsync

    displayClock.setFrequency(60 * 1000);
    mem.getClocks().addClockTarget(-1, displayClock);

    mem.getPIO(1).setUpdateCallback([this](auto time, auto &pio){onPIOUpdate(time, pio);});

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

        auto scrollIndex = meta >> 6;
        auto format = (meta >> 4) & 3;
        auto hRepeat = meta & 0xF;

        // apply scroll offset
        int32_t offset = 0;
        if(scrollIndex)
            offset = *reinterpret_cast<int32_t *>(i2cRegData + 0xF0 + (scrollIndex - 1) * 4);

        addr += offset;

        static const int formatBytes[]{2, 2, 1, 3};

        for(int x = 0; x < hLen; x += hRepeat)
        {
            for(int r = 0; r < hRepeat; r++)
            {
                auto outOff = (x + r + line * vRepeat * hLen) * 3;
                if(format == 0/*?*/ || format == 1) // RGB555
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
        }
    }
}

void PicoVisionBoard::onGPIORead(uint64_t time, GPIO &gpio)
{
    displayUpdate(time);
}

void PicoVisionBoard::onPIOUpdate(uint64_t time, PIO &pio)
{
    auto &txFifo = pio.getTXFIFO(0);
    //auto &smHW = pio.getHW().sm[0];

    if(txFifo.empty())
        return;

    int ramBank = (mem.getGPIO().getPadState() & (1 << 8)) ? 1 : 0;

    while(!txFifo.empty())
    {
        auto data = txFifo.pop();

        bool isResetProg = !ramQuadEnabled[ramBank]; // assume reset program until quad enabled

        if(ramCmdOffset[ramBank] == 0)
        {
            if(isResetProg)
                ramCmdLenWords[ramBank] = data / 32 + 1;
            else
                ramCmdLenWords[ramBank] = (data + 1) * (ramQuadEnabled[ramBank] ? 4 : 1) / 32 + 3;

            ramDataLenBytes[ramBank] = (data + 1) * (ramQuadEnabled[ramBank] ? 4 : 1) / 8;
        }
        else if(ramCmdOffset[ramBank] == 1)
        {
            auto cmd = ramCommand[ramBank] = data >> 24;

            if(cmd == 0x35)
            {
                logf(LogLevel::Debug, logComponent, "psram %i quad enabled", ramBank);
                ramQuadEnabled[ramBank] = true;
            }
            else if(cmd == 0x38) // write
                ramCmdAddr[ramBank] = data & 0xFFFFFF;
            else
                logf(LogLevel::Debug, logComponent, "psram %i cmd %02X %06X len %i", ramBank, cmd, data & 0xFFFFFF, ramDataLenBytes[ramBank]);
        }
        else if(ramCmdOffset[ramBank] == 2 && !isResetProg)
        {
            // pio addr
        }
        else // data
        {
            if(ramCommand[ramBank] == 0x38) // write
            {
                auto offset = (ramCmdOffset[ramBank] - 3) * 4;

                for(int i = 0; i < std::min(4, ramDataLenBytes[ramBank] - offset); i++)
                {
                    uint32_t addr = ramCmdAddr[ramBank] + offset + i;

                    psramData[ramBank][addr] = data >> 24;

                    data <<= 8;
                }
            }
        }

        if(ramCmdOffset[ramBank] == ramCmdLenWords[ramBank])
            ramCmdOffset[ramBank] = 0;
        else
            ramCmdOffset[ramBank]++;
    }
}

void PicoVisionBoard::onI2CWrite(uint64_t time, I2C &i2c, uint8_t data, bool stop)
{
    if(i2c.getTargetAddr() != 0x0D)
        return;

    if(i2cReg == -1)
        i2cReg = data;
    else
    {
        logf(LogLevel::Info, logComponent, "I2C reg %02X = %02X", i2cReg, data);

        // update screen settings on enable
        if(i2cReg == 0xFD && data && !i2cRegData[i2cReg])
            updateScreenSettings();

        i2cRegData[i2cReg++] = data;
    }

    // TODO: need to keep for reads
    if(stop)
        i2cReg = -1;
}