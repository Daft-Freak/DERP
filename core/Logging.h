#pragma once
#include <string_view>

#ifdef __GNUC__
#define printf_attrib(x, y) [[gnu::format(printf, x, y)]]
#else
#define printf_attrib(x, y)
#endif

namespace Logging
{
    enum class Level
    {
        Invalid = -1,

        Debug = 0,
        Info,
        Warning,
        NotImplemented,
        Error,
    };

    enum class Component
    {
        Invalid = -1,
        Other = 0,

        ArmCore,
        Clocks,
        DMA,
        GDB,
        GPIO,
        Main,
        MemoryBus,
        PWM,
        Timer,
        UART,
        USB,
        Watchdog,
    };

    Level stringToLevel(std::string_view str);
    Component stringToComponent(std::string_view str);

    void setEnabled(Level level, bool enabled);
    void setEnabled(Component component, bool enabled);

    printf_attrib(1, 2)
    void logf(const char *format, ...);

    printf_attrib(3, 4)
    void logf(Level level, Component component, const char *format, ...);
}