#pragma once
#include "Board.h"

// default board, has no attached hardware
class PicoBoard final : public Board
{
public:
    PicoBoard(MemoryBus &mem){}

    void getScreenSize(int &w, int &h) override {w = h = 0;}

    bool hasAudio() override {return false;}
    int getNumAudioSamples() override {return 0;}
    int16_t getAudioSample() override {return 0;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override {}
};