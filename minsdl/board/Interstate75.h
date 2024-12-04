#pragma once
#include "Board.h"

class PIO;

class Interstate75Board final : public Board
{
public:
    Interstate75Board(MemoryBus &mem);

    void getScreenSize(int &w, int &h) override;
    int getScreenFormat() override;
    const uint8_t *getScreenData() override;

    bool hasAudio() override {return false;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override {}

private:
    MemoryBus &mem;

    uint32_t screenData[32 * 32];

    int panelWidth = 32;
    int panelHeight = 32;
    
    int row = 0;
    int column = 0;
    bool bottom = false; // bottom/top half

    void onPIOUpdate(uint64_t time, PIO &pio);
};