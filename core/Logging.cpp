#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "Logging.h"

namespace Logging
{
    static uint32_t levelMask = ~0u;
    static uint32_t componentMask = ~0u;

    constexpr uint32_t toMask(Level l){return 1 << static_cast<int>(l);}
    constexpr uint32_t toMask(Component c){return 1 << static_cast<int>(c);}

    static const char *levelToString(Level level)
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

            case Level::Invalid:
            default:
                return "?";
        }
    }

    static const char *componentToString(Component comp)
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
            case Component::GDB:
                return "gdb";
            case Component::GPIO:
                return "gpio";
            case Component::I2C:
                return "i2c";
            case Component::Main:
                return "main";
            case Component::MemoryBus:
                return "membus";
            case Component::PWM:
                return "pwm";
            case Component::Timer:
                return "timer";
            case Component::UART:
                return "uart";
            case Component::USB:
                return "usb";
            case Component::Watchdog:
                return "watchdog";
            
            case Component::Invalid:
            default:
                return "?";
        }
    }

    Level stringToLevel(std::string_view str)
    {
        if(str == "debug")
            return Level::Debug;
        if(str == "info")
            return Level::Info;
        if(str == "warning")
            return Level::Warning;
        if(str == "not implemented" || str == "notimplemented")
            return Level::NotImplemented;
        if(str == "error")
            return Level::Error;

        return Level::Invalid;
    }

    Component stringToComponent(std::string_view str)
    {
        if(str == "other")
            return Component::Other;
        if(str == "armcore")
            return Component::ArmCore;
        if(str == "clocks")
            return Component::Clocks;
        if(str == "dma")
            return Component::DMA;
        if(str == "gdb")
            return Component::GDB;
        if(str == "gpio")
            return Component::GPIO;
        if(str == "i2c")
            return Component::I2C;
        if(str == "main")
            return Component::Main;
        if(str == "membus")
            return Component::MemoryBus;
        if(str == "pwm")
            return Component::PWM;
        if(str == "timer")
            return Component::Timer;
        if(str == "uart")
            return Component::UART;
        if(str == "usb")
            return Component::USB;
        if(str == "watchdog")
            return Component::Watchdog;

        return Component::Invalid;
    }

    void setEnabled(Level level, bool enabled)
    {
        if(level == Level::Invalid)
            return;
        
        if(enabled)
            levelMask |= toMask(level);
        else
            levelMask &= ~toMask(level);
    }

    void setEnabled(Component component, bool enabled)
    {
        if(component == Component::Invalid)
            return;
        
        if(enabled)
            componentMask |= toMask(component);
        else
            componentMask &= ~toMask(component);
    }

    void vlogf(Level level, Component component, const char *format, va_list args)
    {
        if(level == Level::Invalid || component == Component::Invalid)
            return;

        if(!(levelMask & toMask(level)) || !(componentMask & toMask(component)))
            return;

        // get length
        va_list tmp_args;
        va_copy(tmp_args, args);
        int len = vsnprintf(nullptr, 0, format, tmp_args) + 1;
        va_end(tmp_args);

        auto buf = new char[len];
        vsnprintf(buf, len, format, args);
        
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

        va_end(args);
    }

    void logf(Level level, Component component, const char *format, ...)
    {
        va_list args;
        va_start(args, format);

        vlogf(level, component, format, args);

        va_end(args);
    }
}