#include <cstdio>

#include "hardware/platform_defs.h"
#include "hardware/regs/i2c.h"

#include "I2C.h"

#include "MemoryBus.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::I2C;

I2C::I2C(MemoryBus &mem, int index) : mem(mem), index(index)
{
}

void I2C::reset()
{

}

uint32_t I2C::regRead(uint32_t addr)
{
    switch(addr)
    {
    }

    logf(LogLevel::NotImplemented, logComponent, "%i R %04X", index, addr);

    return 0xBADADD55;
}

void I2C::regWrite(uint32_t addr, uint32_t data)
{
    int atomic = addr >> 12;
    addr &= 0xFFF;

    static const char *op[]{" = ", " ^= ", " |= ", " &= ~"};

    switch(addr)
    {
    }

    logf(LogLevel::NotImplemented, logComponent, "%i W %04X%s%08X", index, addr, op[atomic], data);
}
