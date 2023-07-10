#pragma once
#include "Board.h"

class PIO;

class PicoVisionBoard final : public Board
{
public:
    PicoVisionBoard(MemoryBus &mem);

    void getScreenSize(int &w, int &h) override;
    int getScreenFormat() override;
    const uint8_t *getScreenData() override;

    bool hasAudio() override {return false;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override {}

private:
    MemoryBus &mem;

    uint16_t screenData[640 * 480];

    void onPIOUpdate(uint64_t time, PIO &pio);
};