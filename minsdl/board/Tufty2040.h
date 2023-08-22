#pragma once
#include "Board.h"

class PIO;

class Tufty2040Board final : public Board
{
public:
    Tufty2040Board(MemoryBus &mem);

    void getScreenSize(int &w, int &h) override;

    bool hasAudio() override {return false;}
    int getNumAudioSamples() override {return 0;}
    int16_t getAudioSample() override {return 0;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override {}

private:
    MemoryBus &mem;

    int screenDataOff = 0;
    bool doDisplayWrite = false;

    void onPIOUpdate(uint64_t time, PIO &pio);
};