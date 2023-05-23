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
}

void PWM::update(uint64_t target)
{
    auto cycles = clock.getCyclesToTime(target);

    clock.addCycles(cycles);
}

uint64_t PWM::getNextInterruptTime(uint64_t target)
{
    if(!hw.inte)
        return target;

    return target;
}

uint32_t PWM::regRead(uint32_t addr)
{
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

void PWM::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    if(addr < PWM_EN_OFFSET)
    {
        int slice = addr / 20;
        int reg = addr % 20;

        switch(reg)
        {
            case PWM_CH0_CSR_OFFSET:
                updateReg(hw.slice[slice].csr, data, atomic);
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
