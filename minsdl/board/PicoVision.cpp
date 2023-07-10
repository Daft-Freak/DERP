#include <SDL.h>

#include "PicoVision.h"

#include "Logging.h"
#include "MemoryBus.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

PicoVisionBoard::PicoVisionBoard(MemoryBus &mem) : mem(mem)
{
    mem.getGPIO().clearInputFloatingMask(1 << 16); // vsync
    mem.getGPIO().setInputMask(1 << 16); // TODO

    //mem.getPIO(1).setUpdateCallback([this](auto time, auto &pio){onPIOUpdate(time, pio);});
}

void PicoVisionBoard::getScreenSize(int &w, int &h)
{
    // TODO: configurable at runtime
    w = 640;
    h = 480;
}

int PicoVisionBoard::getScreenFormat()
{
    // TODO: also configurable
    return SDL_PIXELFORMAT_RGB555;
}

const uint8_t *PicoVisionBoard::getScreenData()
{
    return reinterpret_cast<uint8_t *>(screenData);
}

void PicoVisionBoard::onPIOUpdate(uint64_t time, PIO &pio)
{
  
}