#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/pwm.h"
#include "hardware/regs/intctrl.h"

#include "PWM.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::PWM;

PWM::PWM(MemoryBus &mem) : mem(mem)
{
}

void PWM::reset()
{
    for(auto &slice : hw.slice)
    {
        slice.csr = PWM_CH0_CSR_RESET;
        slice.div = PWM_CH0_DIV_RESET;
        slice.ctr = PWM_CH0_CTR_RESET;
        slice.cc = PWM_CH0_CC_RESET;
        slice.top = PWM_CH0_TOP_RESET;
    }

    hw.intr = PWM_INTR_RESET;
    hw.inte = PWM_INTE_RESET;
    hw.intf = PWM_INTF_RESET;

    for(unsigned i = 0; i < NUM_PWM_SLICES; i++)
    {
        divCounter[i] = 0;
        ccAInternal[i] = hw.slice[i].cc & PWM_CH0_CC_A_BITS;
        ccBInternal[i] = (hw.slice[i].cc & PWM_CH0_CC_B_BITS) >> PWM_CH0_CC_B_LSB;
        topInternal[i] = hw.slice[i].top;
    }

    outputs = 0;
}

void PWM::update(uint64_t target)
{
    auto cycles = clock.getCyclesToTime(target);

    if(!sliceEnabled && !outputs)
    {
        clock.addCycles(cycles);
        return;
    }

    while(cycles)
    {
        // find update step
        auto step = cycles;
        auto enabled = sliceEnabled;
        for(unsigned i = 0; i < NUM_PWM_SLICES && enabled; i++, enabled >>= 1)
        {
            if(!(enabled & 1))
                continue;

            // first one of TOP or CC A/B that we haven't hit yet
            uint32_t nextCheck = topInternal[i] + 1;
            if(hw.slice[i].ctr < ccAInternal[i] && nextCheck > ccAInternal[i])
                nextCheck = ccAInternal[i];
            if(hw.slice[i].ctr < ccBInternal[i] && nextCheck > ccBInternal[i])
                nextCheck = ccBInternal[i];

            uint32_t cyclesToNext = ((nextCheck - hw.slice[i].ctr - 1) * hw.slice[i].div + (divCounter[i] + 15)) / 16 + 1;

            if(cyclesToNext < step)
                step = cyclesToNext;
        }

        if(!step)
            step = 1;

        clock.addCycles(step);

        auto oldOutputs = outputs;

        // update
        unsigned i = 0;
        enabled = sliceEnabled;
        for(; i < NUM_PWM_SLICES && enabled; i++, enabled >>= 1)
        {
            if(!(enabled & 1))
            {
                outputs &= ~(3 << (i * 2));
                continue;
            }

            divCounter[i] -= step * 16;

            // update counter
            if(divCounter[i] < 0)
            {
                int add = (-divCounter[i] + hw.slice[i].div - 1) / hw.slice[i].div;
                divCounter[i] += hw.slice[i].div * add;
                hw.slice[i].ctr += add;
            }

            // check thresholds
            if(hw.slice[i].ctr > topInternal[i])
            {
                // wrap
                hw.slice[i].ctr = 0;

                // reload internal cc/top
                ccAInternal[i] = hw.slice[i].cc & PWM_CH0_CC_A_BITS;
                ccBInternal[i] = (hw.slice[i].cc & PWM_CH0_CC_B_BITS) >> PWM_CH0_CC_B_LSB;
                topInternal[i] = hw.slice[i].top;

                // go high
                outputs |= 3 << (i * 2);
            }
            else
            {
                if(hw.slice[i].ctr == ccAInternal[i])
                {
                    // go low A
                    outputs &= ~(1 << (i * 2));
                }

                if(hw.slice[i].ctr == ccBInternal[i])
                {
                    // go low B
                    outputs &= ~(2 << (i * 2));
                }
            }
        }

        // clear any remaining slices
        if(i < NUM_PWM_SLICES && outputs)
            outputs &= ~(0xFFFF << (i * 2));

        if(outputs != oldOutputs)
        {
            mem.getGPIO().update(clock.getTime());
            mem.getGPIO().setFuncOutputs(GPIO::Function::PWM, outputs | outputs << 16);
            mem.getGPIO().setFuncOutputEnables(GPIO::Function::PWM, ~0u); // TODO?

            if(outputCallback && ((outputs ^ oldOutputs) & outputCallbackMask))
                outputCallback(clock.getTime(), outputs);
        }

        cycles -= step;
    }
}

void PWM::setOutputCallback(OutputCallback cb, uint16_t mask)
{
    outputCallback = cb;
    outputCallbackMask = mask;
}

uint64_t PWM::getNextInterruptTime(uint64_t target)
{
    if(!hw.inte)
        return target;

    return target;
}

uint32_t PWM::regRead(uint64_t time, uint32_t addr)
{
    update(time);

    if(addr < PWM_EN_OFFSET)
    {
        int slice = addr / 20;
        int reg = addr % 20;

        switch(reg)
        {
            case PWM_CH0_CSR_OFFSET:
                return hw.slice[slice].csr;
            case PWM_CH0_DIV_OFFSET:
                return hw.slice[slice].div;
            case PWM_CH0_CTR_OFFSET:
                return hw.slice[slice].ctr;
            case PWM_CH0_CC_OFFSET:
                return hw.slice[slice].cc;
            case PWM_CH0_TOP_OFFSET:
                return hw.slice[slice].top;
        }
    }

    switch(addr)
    {
    }

    logf(LogLevel::NotImplemented, logComponent, "R %04X", addr);
    return 0;
}

void PWM::regWrite(uint64_t time, uint32_t addr, uint32_t data)
{
    update(time);

    int atomic = addr >> 12;
    addr &= 0xFFF;

    if(addr < PWM_EN_OFFSET)
    {
        int slice = addr / 20;
        int reg = addr % 20;

        switch(reg)
        {
            case PWM_CH0_CSR_OFFSET:
                if(updateReg(hw.slice[slice].csr, data, atomic))
                {
                    if(hw.slice[slice].csr & ~PWM_CH0_CSR_EN_BITS)
                        logf(LogLevel::NotImplemented, logComponent, "W CH%i_CSR = %08X", slice, data);

                    if(hw.slice[slice].csr & PWM_CH0_CSR_EN_BITS)
                        sliceEnabled |= 1 << slice;
                    else
                        sliceEnabled &= ~(1 << slice);
                }
                return;
            case PWM_CH0_DIV_OFFSET:
                updateReg(hw.slice[slice].div, data, atomic);
                return;
            case PWM_CH0_CTR_OFFSET:
                updateReg(hw.slice[slice].ctr, data, atomic);
                return;
            case PWM_CH0_CC_OFFSET:
                updateReg(hw.slice[slice].cc, data, atomic);
                return;
            case PWM_CH0_TOP_OFFSET:
                updateReg(hw.slice[slice].top, data, atomic);
                return;
        }
    }

    switch(addr)
    {
    }

    logf(LogLevel::NotImplemented, logComponent, "W %04X = %08X", addr, data);
}
