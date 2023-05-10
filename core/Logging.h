#pragma once

#ifdef __GNUC__
#define printf_attrib(x, y) [[gnu::format(printf, x, y)]]
#else
#define printf_attrib(x, y)
#endif

namespace Logging
{
    enum class Level
    {
        Debug,
        Info,
        Warning,
        NotImplemented,
        Error,
    };

    enum class Component
    {
        Other,

        ArmCore,
        Clocks,
        DMA,
        GPIO,
        Main,
        MemoryBus,
        Timer,
        UART,
        USB,
        Watchdog,
    };

    constexpr int toMask(Level l){return 1 << static_cast<int>(l);}
    constexpr int toMask(Component c){return 1 << static_cast<int>(c);}

    printf_attrib(1, 2)
    void logf(const char *format, ...);

    printf_attrib(3, 4)
    void logf(Level level, Component component, const char *format, ...);
}