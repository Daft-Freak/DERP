#pragma once
#include "Board.h"

#include "ClockTarget.h"

class GPIO;
class PIO;

class PicoSystemBoard final : public Board
{
public:
    PicoSystemBoard(MemoryBus &mem, bool picosystemSDK);

    void getScreenSize(int &w, int &h) override;
    int getScreenFormat() override;
    const uint8_t *getScreenData() override;

    bool hasAudio() override;
    int getNumAudioSamples() override;
    int16_t getAudioSample() override;

    void handleEvent(SDL_Event &event) override;
    void update(uint64_t time) override;

private:
    void handleDisplayCommandData(uint8_t data);

    MemoryBus &mem;
    bool isPicoSystemSDK;

    uint32_t buttonState = 0;

    uint16_t screenData[240 * 240];
    unsigned int displayScanline = 0;
    ClockTarget displayClock;
    int screenDataOff = 0;

    int displayCommand = -1;
    int displayCommandOff = 0;
    uint8_t displayCommandData[4];

    ClockTarget audioClock;
    bool lastAudioVal = false;
    static const int audioBufferSize = 1024;
    volatile int audioReadOff = 0, audioWriteOff = 0;
    int16_t audioSamples[audioBufferSize]{};

    void displayUpdate(uint64_t time, bool forIntr = false);
    void audioUpdate(uint64_t time);

    void onGPIORead(uint64_t time, GPIO &gpio);

    void onInterruptUpdate(uint64_t time, uint32_t irqMask);

    uint64_t onGetNextInterruptTime(uint64_t time);

    void onPWMUpdate(uint64_t time, uint16_t pwm);
    void onPIOUpdate(uint64_t time, PIO &pio);
};