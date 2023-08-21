#pragma once
#include <cstdint>

class MemoryBus;

class Board
{
public:
    virtual ~Board() = default;

    virtual void getScreenSize(int &w, int &h) = 0;

    virtual bool hasAudio() = 0;
    virtual int getNumAudioSamples() = 0;
    virtual int16_t getAudioSample() =  0;

    virtual void update(uint64_t time) = 0;
private:
};