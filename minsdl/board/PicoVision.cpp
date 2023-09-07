#include <SDL.h>

#include "PicoVision.h"

#include "Logging.h"
#include "MemoryBus.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

PicoVisionBoard::PicoVisionBoard(MemoryBus &mem) : mem(mem)
{
    mem.getGPIO().setReadCallback([this](auto time, auto &gpio){onGPIORead(time, gpio);});
    mem.getGPIO().clearInputFloatingMask(1 << 16); // vsync

    displayClock.setFrequency(60 * 1000);
    mem.getClocks().addClockTarget(-1, displayClock);

    mem.getPIO(1).setUpdateCallback([this](auto time, auto &pio){onPIOUpdate(time, pio);});
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