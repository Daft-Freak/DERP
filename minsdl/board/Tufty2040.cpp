#include <SDL.h>

#include "Tufty2040.h"

#include "Logging.h"
#include "MemoryBus.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

Tufty2040Board::Tufty2040Board(MemoryBus &mem, const Options &options) : mem(mem)
{
    mem.getPIO(1).setTXCallback([this](auto time, auto &pio, auto sm, auto data){onPIOTX(time, pio, sm, data);});
}

void Tufty2040Board::getScreenSize(int &w, int &h)
{
    w = 320;
    h = 240;
}

int Tufty2040Board::getScreenFormat()
{
    return SDL_PIXELFORMAT_RGB565;
}

const uint8_t *Tufty2040Board::getScreenData()
{
    return screenData;
}

void Tufty2040Board::onPIOTX(uint64_t time, PIO &pio, int sm, uint32_t data)
{
    if(sm != 0)
        return;

    bool rs = mem.getGPIO().getPadState() & (1 << 11);

    data &= 0xFF;

    if(!rs)
    {
        doDisplayWrite = false;

        // RAM write command
        if(data == 0x2C)
        {
            screenDataOff = 0;
            doDisplayWrite = true;
        }
        else if(data)
            logf(LogLevel::Debug, logComponent, "tf display cmd %02X", data);
    }
    else if(doDisplayWrite)
    {
        screenData[screenDataOff ^ 1] = data & 0xFF; // byte swap
        screenDataOff++;

        if(screenDataOff == 320 * 240 * 2)
            screenDataOff = 0;
    }
}
