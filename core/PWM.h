#pragma once

#include <cstdint>
#include <functional>

#include "hardware/structs/pwm.h"

#include "ClockTarget.h"

class MemoryBus;

class PWM final : public ClockedDevice
{
public:
    using OutputCallback = std::function<void(uint64_t, uint16_t)>;

    PWM(MemoryBus &mem);

    void reset();

    void update(uint64_t target);
    bool needUpdateForInterrupts()
    {
        return hw.inte;
    }

    void setOutputCallback(OutputCallback cb, uint16_t mask);

    uint64_t getNextInterruptTime(uint64_t target);

    uint32_t regRead(uint64_t time, uint32_t addr);
    void regWrite(uint64_t time, uint32_t addr, uint32_t data);

    ClockTarget &getClock() {return clock;}

    int getDeviceFlags() const {return 0;}

private:
    MemoryBus &mem;

    ClockTarget clock;

    pwm_hw_t hw;

    int divCounter[NUM_PWM_SLICES];

    uint16_t ccAInternal[NUM_PWM_SLICES];
    uint16_t ccBInternal[NUM_PWM_SLICES];
    uint16_t topInternal[NUM_PWM_SLICES];

    uint8_t sliceEnabled = 0;

    uint16_t outputs;

    OutputCallback outputCallback;
    uint16_t outputCallbackMask = 0;
};
