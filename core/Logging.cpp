#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "Logging.h"

namespace Logging
{
    const char *levelToString(Level level)
    {
        switch(level)
        {
            case Level::Debug:
                return "debug";
            case Level::Info:
                return "info";
            case Level::Warning:
                return "warning";
            case Level::NotImplemented:
                return "not implemented";
            case Level::Error:
                return "error";
        }

        return "?";
    }

    const char *componentToString(Component comp)
    {
        switch(comp)
        {
            case Component::Other:
                return "other";
            case Component::ArmCore:
                return "armcore";
            case Component::Clocks:
                return "clocks";
            case Component::DMA:
                return "dma";
            case Component::GPIO:
                return "gpio";
            case Component::MemoryBus:
                return "membus";
            case Component::Timer:
                return "timer";
            case Component::UART:
                return "uart";
            case Component::Watchdog:
                return "watchdog";
        }

        return "?";
    }


    void vlogf(Level level, Component component, const char *format, va_list args)
    {
        // get length
        va_list tmp_args;
        va_copy(tmp_args, args);
        int len = vsnprintf(nullptr, 0, format, tmp_args) + 1;
        va_end(tmp_args);

        auto buf = new char[len];
        vsnprintf(buf, len, format, args);
        va_end(args);
        
        auto levelStr = levelToString(level);
        auto compStr = componentToString(component);

        int levelPad = 16 - strlen(levelStr);
        int compPad = 9 - strlen(compStr);

        printf("[%s]%*s[%s]%*s%s\n", levelStr, levelPad, "", compStr, compPad, "", buf);

        delete[] buf;
    }

    void logf(const char *format, ...)
    {
        va_list args;
        va_start(args, format);

        vlogf(Level::Info, Component::Other, format, args);
    }

    void logf(Level level, Component component, const char *format, ...)
    {
        va_list args;
        va_start(args, format);

        vlogf(level, component, format, args);
    }
}