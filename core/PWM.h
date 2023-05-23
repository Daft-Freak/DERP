#pragma once

#include <cstdint>

#include "hardware/structs/pwm.h"

#include "ClockTarget.h"

class MemoryBus;

class PWM final
{
public:
    PWM(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    void updateForInterrupts(uint64_t target)
    {
        if(hw.inte)
            update(target);
    }

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint32_t addr);
    void regWrite(uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

private:
    MemoryBus &mem;

    ClockTarget clock;

    pwm_hw_t hw;

    int divCounter[NUM_PWM_SLICES];

    uint16_t ccAInternal[NUM_PWM_SLICES];
    uint16_t ccBInternal[NUM_PWM_SLICES];
    uint16_t topInternal[NUM_PWM_SLICES];

    uint16_t outputs;
};
