#pragma once
#include <cstdint>

class MemoryBus;
union SDL_Event;

class Board
{
public:
    virtual ~Board() = default;

    virtual void getScreenSize(int &w, int &h) = 0;
    virtual const uint8_t *getScreenData() = 0;

    virtual bool hasAudio() = 0;
    virtual int getNumAudioSamples() = 0;
    virtual int16_t getAudioSample() =  0;

    virtual void handleEvent(SDL_Event &event) = 0;
    virtual void update(uint64_t time) = 0;
private:
};