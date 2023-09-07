#pragma once
#include "Board.h"

#include "ClockTarget.h"

class GPIO;
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
    void update(uint64_t time) override;

private:
    MemoryBus &mem;

    uint16_t screenData[640 * 480];

    ClockTarget displayClock;
    int displayTick = 0;

    int ramCmdOffset[2] = {0, 0};
    int ramCmdLenWords[2] = {0, 0};
    int ramDataLenBytes[2] = {0, 0};
    uint8_t ramCommand[2] = {0, 0};
    uint32_t ramCmdAddr[2] = {0, 0};

    bool ramQuadEnabled[2] = {0, 0};

    uint8_t psramData[2][8 * 1024 * 1024];

    void displayUpdate(uint64_t time, bool forIntr = false);

    void onGPIORead(uint64_t time, GPIO &gpio);

    void onPIOUpdate(uint64_t time, PIO &pio);
};