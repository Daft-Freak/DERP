#pragma once
#include "Board.h"

class PIO;

class Tufty2040Board final : public Board
{
public:
    Tufty2040Board(MemoryBus &mem);

    void getScreenSize(int &w, int &h) override;
    int getScreenFormat() override;
    const uint8_t *getScreenData() override;

    bool hasAudio() override {return false;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override {}

private:
    MemoryBus &mem;

    uint8_t screenData[320 * 240 * 2];
    int screenDataOff = 0;
    bool doDisplayWrite = false;

    void onPIOTX(uint64_t time, PIO &pio, int sm, uint32_t data);
};