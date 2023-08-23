#include "Tufty2040.h"

#include "Logging.h"
#include "MemoryBus.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

Tufty2040Board::Tufty2040Board(MemoryBus &mem) : mem(mem)
{
    mem.getPIO(1).setUpdateCallback([this](auto time, auto &pio){onPIOUpdate(time, pio);});
}

void Tufty2040Board::getScreenSize(int &w, int &h)
{
    w = 320;
    h = 240;
}

const uint8_t *Tufty2040Board::getScreenData()
{
    return screenData;
}

void Tufty2040Board::onPIOUpdate(uint64_t time, PIO &pio)
{
    // display is usually PIO1 SM0
    auto &txFifo = pio.getTXFIFO(0);

    bool rs = mem.getGPIO().getPadState() & (1 << 11);

    if(txFifo.empty())
        return;

    while(!txFifo.empty())
    {
        auto data = txFifo.pop() & 0xFF;
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

    pio.updateFifoStatus();
}