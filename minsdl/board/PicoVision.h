#pragma once
#include "Board.h"

#include "../Main.h"

#include "ClockTarget.h"

class GPIO;
class I2C;
class PIO;

class PicoVisionBoard final : public Board
{
public:
    PicoVisionBoard(MemoryBus &mem, const Options &options);

    void getScreenSize(int &w, int &h) override;
    int getScreenFormat() override;
    const uint8_t *getScreenData() override;

    bool hasAudio() override {return false;}

    void handleEvent(SDL_Event &event) override {}
    void update(uint64_t time) override;

private:
    MemoryBus &mem;

    uint8_t screenData[800 * 600 * 3];

    ClockTarget displayClock;
    int displayTick = 0;

    int ramCmdOffset[2] = {0, 0};
    int ramCmdLenWords[2] = {0, 0};
    int ramDataLenBytes[2] = {0, 0};
    uint8_t ramCommand[2] = {0, 0};
    uint32_t ramCmdAddr[2] = {0, 0};

    bool ramQuadEnabled[2] = {0, 0};

    uint8_t psramData[2][8 * 1024 * 1024];

    int i2cReg = -1;
    int i2cRegIndex = 0;

    uint8_t i2cRegData[256]{};

    uint8_t scrollGroupData[7 * 13];

    void displayUpdate(uint64_t time, bool forIntr = false);
    void updateScreenData();

    void onGPIORead(uint64_t time, GPIO &gpio);

    void onInterruptUpdate(uint64_t time, uint32_t irqMask);
    uint64_t onGetNextInterruptTime(uint64_t time);

    void onPIOTX(uint64_t time, PIO &pio, int sm, uint32_t data);

    void onI2CWrite(uint64_t time, I2C &i2c, uint8_t data, bool stop);
};