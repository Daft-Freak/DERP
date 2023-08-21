#pragma once
#include "Board.h"

// default board, has no attached hardware
class PicoBoard final : public Board
{
public:
    PicoBoard(MemoryBus &mem){}

    void getScreenSize(int &w, int &h) override {w = h = 0;}
    bool hasAudio() override {return false;}

    void update(uint64_t time) override {}
};