#include <SDL.h>

#include "PicoSystem.h"

#include "Logging.h"
#include "MemoryBus.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Board;

static const std::unordered_map<SDL_Keycode, int> picosystemKeyMap {
    {SDLK_RIGHT,  1 << 21},
    {SDLK_LEFT,   1 << 22},
    {SDLK_UP,     1 << 23},
    {SDLK_DOWN,   1 << 20},

    {SDLK_z,      1 << 18},
    {SDLK_x,      1 << 19},
    {SDLK_c,      1 << 17},
    {SDLK_v,      1 << 16},
};

PicoSystemBoard::PicoSystemBoard(MemoryBus &mem, bool picosystemSDK) : mem(mem), isPicoSystemSDK(picosystemSDK)
{
    auto &clocks = mem.getClocks();

    mem.setInterruptUpdateCallback([this](auto time, auto irqMask){onInterruptUpdate(time, irqMask);});
    mem.setGetNextInterruptTimeCallback([this](auto time){return onGetNextInterruptTime(time);});

    int buttonMask = 0xFF0000;

    mem.getGPIO().setReadCallback([this](auto time, auto &gpio){onGPIORead(time, gpio);});
    mem.getGPIO().clearInputFloatingMask(1 << 8/*TE*/ | buttonMask); // buttons pulled high on board

    mem.getPIO(0).setUpdateCallback([this](auto time, auto &pio){onPIOUpdate(time, pio);});

    mem.getPWM().setOutputCallback([this](auto time, auto pwm){onPWMUpdate(time, pwm);}, 1 << 11); // audio

    const int fps = picosystemSDK ? 40 : 50;
    displayClock.setFrequency(fps * 240);
    clocks.addClockTarget(-1, displayClock);

    audioClock.setFrequency(48000);
    clocks.addClockTarget(-1, audioClock);
}

void PicoSystemBoard::getScreenSize(int &w, int &h)
{
    w = 240;
    h = 240;
}

int PicoSystemBoard::getScreenFormat()
{
    // TODO: implement screen registers instead of guessing
    if(isPicoSystemSDK)
        return SDL_PIXELFORMAT_ARGB4444;
    else // assume 32blit-sdk
        return SDL_PIXELFORMAT_BGR565;
}

const uint8_t *PicoSystemBoard::getScreenData()
{
    return reinterpret_cast<uint8_t *>(screenData);
}

bool PicoSystemBoard::hasAudio()
{
    return true;
}

int PicoSystemBoard::getNumAudioSamples()
{
    int avail = audioWriteOff - audioReadOff;
    if(avail < 0)
        avail += audioBufferSize;

    return avail;
}

int16_t PicoSystemBoard::getAudioSample()
{
    if(!getNumAudioSamples())
        return 0;

    auto val = audioSamples[audioReadOff++];
    audioReadOff %= audioBufferSize;

    return val;
}

void PicoSystemBoard::handleEvent(SDL_Event &event)
{
    switch(event.type)
    {
        case SDL_KEYDOWN:
        {
            auto it = picosystemKeyMap.find(event.key.keysym.sym);
            if(it != picosystemKeyMap.end())
                buttonState |= it->second;
            break;
        }
        case SDL_KEYUP:
        {
            auto it = picosystemKeyMap.find(event.key.keysym.sym);
            if(it != picosystemKeyMap.end())
                buttonState &= ~it->second;
            break;
        }

    }
}

void PicoSystemBoard::update(uint64_t time)
{
    displayUpdate(time);
    audioUpdate(time);
}

void PicoSystemBoard::displayUpdate(uint64_t time, bool forIntr)
{
    auto lines = displayClock.getCyclesToTime(time);

    if(!lines)
        return;

    while(lines)
    {
        int step = std::max(1u, std::min(lines, 239 - displayScanline));
        lines -= step;

        displayClock.addCycles(step);

        // set TE when we're on the last scanline
        // TODO: use STE reg
        auto newLine = (displayScanline + step) % 240;

        if(newLine == 239) // now last line
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().setInputMask(1 << 8);
        }
        else if(displayScanline == 239) // was last line
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().clearInputMask(1 << 8);
        }

        displayScanline = newLine;
    }
}

void PicoSystemBoard::audioUpdate(uint64_t time)
{
    auto samples = audioClock.getCyclesToTime(time);

    for(uint32_t i = 0; i < samples; i++)
    {
        while((audioWriteOff + 1) % audioBufferSize == audioReadOff);

        int level = 0x1000;
        audioSamples[audioWriteOff++] = lastAudioVal ? level : -level;
        audioWriteOff %= audioBufferSize;
    }

    audioClock.addCycles(samples);
}

void PicoSystemBoard::onGPIORead(uint64_t time, GPIO &gpio)
{
    // apply buttons
    int buttonMask = 0xFF0000;

    gpio.setInputMask(~buttonState & buttonMask); // not pressed -> pulled high
    gpio.clearInputMask(buttonState); // pressed -> pulled down

    displayUpdate(time);
}

void PicoSystemBoard::onInterruptUpdate(uint64_t time, uint32_t irqMask)
{
    if(irqMask & (1 << 13) /*IO_IRQ_BANK0*/)
        displayUpdate(time, true);
}

uint64_t PicoSystemBoard::onGetNextInterruptTime(uint64_t time)
{
    if(!mem.getGPIO().interruptsEnabledOnPin(8))
        return time;

    int lines = std::max(1u, 239 - displayScanline);
    auto ret = displayClock.getTimeToCycles(lines);

    return ret;
}

void PicoSystemBoard::onPWMUpdate(uint64_t time, uint16_t pwm)
{
    audioUpdate(time);
    lastAudioVal = pwm & (1 << 11);
}

void PicoSystemBoard::onPIOUpdate(uint64_t time, PIO &pio)
{
    // display is usually PIO0 SM0
    auto &txFifo = pio.getTXFIFO(0);
    auto &smHW = pio.getHW().sm[0];

    if(txFifo.empty())
        return;

    while(!txFifo.empty())
    {
        auto data = txFifo.pop();

        if(smHW.shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS)
        {
            // 32blit-sdk hires or command
            auto pullThresh = (smHW.shiftctrl & PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS) >> PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB;
            if(pullThresh == 8)
            {
                // commands
                logf(LogLevel::Debug, logComponent, "ps display cmd %02X", data & 0xFF);
                screenDataOff = 0;
            }
            else
            {
                // hires data
                screenData[screenDataOff++] = data & 0xFFFF;

                if(screenDataOff == 240 * 240)
                    screenDataOff = 0;
            }
        }
        else
        {
            // 32blit-sdk lores or picosystem-sdk
            bool lores = true;

            if(isPicoSystemSDK)
            {
                // picosystem sdk un-swaps in the pio program
                data = data >> 16 | data << 16;

                // try to work out if this is hires mode
                int top = (smHW.execctrl & PIO_SM0_EXECCTRL_WRAP_TOP_BITS) >> PIO_SM0_EXECCTRL_WRAP_TOP_LSB;
                int bottom = (smHW.execctrl & PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS) >> PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB;
            
                if(top - bottom < 12) // hires program is shorter (8 vs 18 instrs)
                    lores = false;
            }

            // FIXME: picosystem-sdk hires
            
            screenData[screenDataOff++] = data & 0xFFFF;
            if(lores)
                screenData[screenDataOff++] = data & 0xFFFF;
            screenData[screenDataOff++] = data >> 16;
            if(lores)
                screenData[screenDataOff++] = data >> 16;
        
            if(screenDataOff == 240 * 240)
                screenDataOff = 0;
        }
    }

    pio.updateFifoStatus();
}