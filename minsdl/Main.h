#pragma once
#include <string>
#include <cstdint>

enum class BoardId
{
    Unknown = -1,
    Pico = 0,

    PimoroniInterstate75,
    PimoroniPicoSystem,
    PimoroniPicoVision,
    PimoroniTufty2040,
};

struct Options
{
    int screenScale = 5;
    BoardId boardId = BoardId::Unknown;

    bool usbEnabled = false;
    bool usbipEnabled = false;
    bool gdbEnabled = false;
    
    std::string ioLogPath;
};

void updateScreenSettings();

void queueAudio(const int16_t *samples, unsigned int numSamples);