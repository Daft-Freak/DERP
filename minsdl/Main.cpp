#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include <SDL.h>

#include "ARMv6MCore.h"
#include "GDBServer.h"
#include "Logging.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Main;

enum class Board
{
    Unknown = -1,
    Pico = 0,
    PimoroniPicoSystem,
    PimoroniTufty2040,
};

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpuCores[2]{mem, mem};

static GDBServer gdbServer;

static uint8_t bootROM[0x4000];

static std::ifstream uf2File;

static Board board = Board::Unknown;
static bool picosystemSDK = false;

uint16_t screenData[320 * 240];

static uint32_t buttonState = 0;
static unsigned int displayScanline = 0;
static ClockTarget displayClock;
static int screenDataOff = 0;

static ClockTarget audioClock;
static bool lastAudioVal = false;
static const int audioBufferSize = 1024;
static volatile int audioReadOff = 0, audioWriteOff = 0;
static int16_t audioSamples[audioBufferSize]{};

static const uint32_t uf2MagicStart0 = 0x0A324655, uf2MagicStart1 = 0x9E5D5157, uf2MagicEnd = 0x0AB16F30;

struct UF2Block
{
    uint32_t magicStart[2];
    uint32_t flags;
    uint32_t addr;
    uint32_t payloadSize;
    uint32_t blockNo;
    uint32_t numBlocks;
    uint32_t familyIdSize;
    uint8_t payload[476];
    uint32_t magicEnd;
};
static_assert(sizeof(UF2Block) == 512);

static const std::unordered_map<SDL_Keycode, int> picosystemKeyMap {
    {SDLK_RIGHT,  1 << 21},
    {SDLK_LEFT,   1 << 22},
    {SDLK_UP,     1 << 23},
    {SDLK_DOWN,   1 << 20},

    {SDLK_z,      1 << 18},
    {SDLK_x,      1 << 19},
    {SDLK_c,      1 << 17},
    {SDLK_v,      1 << 16},
};

static Board stringToBoard(std::string_view str)
{
    if(str == "pico")
        return Board::Pico;

    if(str == "pimoroni_picosystem")
        return Board::PimoroniPicoSystem;

    if(str == "pimoroni_tufty2040")
        return Board::PimoroniTufty2040;

    logf(LogLevel::Warning, logComponent, "Unknown board \"%.*s\", falling back to \"pico\"", int(str.length()), str.data()); 
    return Board::Pico;
}

static void getBoardScreenSize(Board board, int &w, int &h)
{
    switch(board)
    {
        case Board::PimoroniPicoSystem:
            w = 240;
            h = 240;
            break;
        case Board::PimoroniTufty2040:
            w = 320;
            h = 240;
            break;

        default:
            w = 0;
            h = 0;
    }
}

static bool parseUF2(std::ifstream &file)
{
    while(!file.eof())
    {
        UF2Block block;
        file.read(reinterpret_cast<char *>(&block), sizeof(block));

        if(block.magicStart[0] != uf2MagicStart0 || block.magicStart[1] != uf2MagicStart1 || block.magicEnd != uf2MagicEnd)
        {
            logf(LogLevel::Error, logComponent, "Bad UF2 block magic!");
            return false;
        }

        auto ptr = mem.mapAddress(block.addr);
        if(ptr)
            memcpy(ptr, block.payload, block.payloadSize);
        else
            logf(LogLevel::Warning, logComponent, "Can't write UF2 payload to %08X!", block.addr);
    }

    // parse binary info
    const uint32_t biStart = 0x7188EBF2, biEnd = 0xE71AA390;

    const uint16_t biTypeIdAndString = 6;

    const uint16_t biTagRaspberryPi = 0x5052;

    static const std::map<uint32_t, const char *> raspberryPiIdMap{
        {0x02031c86, "program_name"},
        {0x11a9bc3a, "program_version"},
        {0x9da22254, "program_build_date"},
        {0x68f465de, "program_binary_end"},
        {0x1856239a, "program_url"},
        {0xb6a07c19, "program_description"},
        {0xa1f4b453, "program_feature"},
        {0x4275f0d3, "program_build_attribute"},
        {0x5360b3ab, "sdk_version"},
        {0xb63cffbb, "pico_board"},
        {0x7f8882e1, "boot2_name"}};

    auto flash = mem.mapAddress(0x10000000);

    auto ptr = reinterpret_cast<uint32_t *>(flash + 256);
    uint32_t infoStartAddr = 0, infoEndAddr;

    for (int i = 0; i < 256 / 4; i++, ptr++)
    {
        if (ptr[0] == biStart && ptr[4] == biEnd)
        {
            infoStartAddr = ptr[1];
            infoEndAddr = ptr[2];
            // infoMapAddr = ptr[3];
        }
    }

    if (infoStartAddr)
    {
        logf(LogLevel::Info, logComponent, "binary_info:");

        for (auto addr = infoStartAddr; addr < infoEndAddr; addr += 4)
        {
            auto infoPtr = *reinterpret_cast<uint32_t *>(flash + addr - 0x10000000);
            auto infoData = flash + infoPtr - 0x10000000;

            auto type = *reinterpret_cast<uint16_t *>(infoData);
            auto tag = *reinterpret_cast<uint16_t *>(infoData + 2);

            if (tag != biTagRaspberryPi)
                continue;

            if (type == biTypeIdAndString)
            {
                auto id = *reinterpret_cast<uint32_t *>(infoData + 4);
                auto charPtr = *reinterpret_cast<uint32_t *>(infoData + 8);
                auto str = reinterpret_cast<char *>(flash + charPtr - 0x10000000);

                auto idStr = raspberryPiIdMap.find(id);
                if(idStr != raspberryPiIdMap.end())
                    logf(LogLevel::Info, logComponent, "\t%s: %s", idStr->second, str);

                // detect board
                if(id == 0xb63cffbb/*pico_board*/ && board == Board::Unknown)
                    board = stringToBoard(str);
            }
        }
    }

    return true;
}

// picosystem external hardware/IO
static void displayUpdate(uint64_t time, bool forIntr = false)
{
    auto lines = displayClock.getCyclesToTime(time);

    if(!lines)
        return;

    while(lines)
    {
        int step = std::max(1u, std::min(lines, 239 - displayScanline));
        lines -= step;

        displayClock.addCycles(step);

        // set TE when we're on the last scanline
        // TODO: use STE reg
        auto newLine = (displayScanline + step) % 240;

        if(newLine == 239) // now last line
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().setInputMask(1 << 8);
        }
        else if(displayScanline == 239) // was last line
        {
            if(forIntr)
                mem.gpioUpdate(displayClock.getTime());
            mem.getGPIO().clearInputMask(1 << 8);
        }

        displayScanline = newLine;
    }
}

static int getNumAudioSamples()
{
    int avail = audioWriteOff - audioReadOff;
    if(avail < 0)
        avail += audioBufferSize;

    return avail;
}

static void audioUpdate(uint64_t time)
{
    auto samples = audioClock.getCyclesToTime(time);

    for(uint32_t i = 0; i < samples; i++)
    {
        while((audioWriteOff + 1) % audioBufferSize == audioReadOff);

        int level = 0x1000;
        audioSamples[audioWriteOff++] = lastAudioVal ? level : -level;
        audioWriteOff %= audioBufferSize;
    }

    audioClock.addCycles(samples);
}

static void onGPIORead(uint64_t time, GPIO &gpio)
{
    // apply buttons
    int buttonMask = 0xFF0000;

    gpio.setInputFloatingMask(~buttonState & buttonMask); // not pressed -> floating
    gpio.clearInputFloatingMask(buttonState); // pressed -> pulled down

    displayUpdate(time);
}

static void onInterruptUpdate(uint64_t time, uint32_t irqMask)
{
    if(irqMask & (1 << 13) /*IO_IRQ_BANK0*/)
        displayUpdate(time, true);
}

static uint64_t onGetNextInterruptTime(uint64_t time)
{
    if(!mem.getGPIO().interruptsEnabledOnPin(8))
        return time;

    int lines = std::max(1u, 239 - displayScanline);
    auto ret = displayClock.getTimeToCycles(lines);

    return ret;
}

static void onPWMUpdate(uint64_t time, uint16_t pwm)
{
    audioUpdate(time);
    lastAudioVal = pwm & (1 << 11);
}

static void onPIOUpdate(uint64_t time, PIO &pio)
{
    // display is usually PIO0 SM0
    auto &txFifo = pio.getTXFIFO(0);
    auto &smHW = pio.getHW().sm[0];

    if(txFifo.empty())
        return;

    while(!txFifo.empty())
    {
        auto data = txFifo.pop();

        if(smHW.shiftctrl & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS)
        {
            // 32blit-sdk hires or command
            auto pullThresh = (smHW.shiftctrl & PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS) >> PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB;
            if(pullThresh == 8)
            {
                // commands
                logf(LogLevel::Debug, logComponent, "ps display cmd %02X", data & 0xFF);
                screenDataOff = 0;
            }
            else
            {
                // hires data
                screenData[screenDataOff++] = data & 0xFFFF;

                if(screenDataOff == 240 * 240)
                    screenDataOff = 0;
            }
        }
        else
        {
            // 32blit-sdk lores or picosystem-sdk

            // picosystem sdk un-swaps in the pio program
            if(picosystemSDK)
                data = data >> 16 | data << 16;

            // FIXME: picosystem-sdk hires
            
            screenData[screenDataOff++] = data & 0xFFFF;
            screenData[screenDataOff++] = data & 0xFFFF;
            screenData[screenDataOff++] = data >> 16;
            screenData[screenDataOff++] = data >> 16;
            if(screenDataOff == 240 * 240)
                screenDataOff = 0;
        }
    }

    pio.updateFifoStatus();
}

static void runGDBServer()
{
    gdbServer.setCPUs(cpuCores, 2);

    if(!gdbServer.start())
        logf(LogLevel::Error, logComponent, "Failed to start GDB server!");

    while(!quit)
    {
        if(!gdbServer.update(true))
           logf(LogLevel::Error, logComponent, "Failed to update GDB server!");
    }
}

static void audioCallback(void *userdata, Uint8 *stream, int len)
{
    auto ptr = reinterpret_cast<int16_t *>(stream);
    for(int i = 0; i < len / 2; i++)
    {
        while(!getNumAudioSamples() && !quit);

        *ptr++ = audioSamples[audioReadOff++];
        audioReadOff %= audioBufferSize;
    }
}

static void pollEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
            case SDL_KEYDOWN:
            {
                auto it = picosystemKeyMap.find(event.key.keysym.sym);
                if(it != picosystemKeyMap.end())
                    buttonState |= it->second;
                break;
            }
            case SDL_KEYUP:
            {
                auto it = picosystemKeyMap.find(event.key.keysym.sym);
                if(it != picosystemKeyMap.end())
                    buttonState &= ~it->second;
                break;
            }
            case SDL_QUIT:
                quit = true;
                break;
        }
    }
}

static void handleLogArg(const char *arg)
{
    bool enable = true;
    if(arg[0] == '-')
    {
        enable = false;
        arg++;
    }

    std::string_view str(arg);

    auto level = Logging::stringToLevel(str);
    if(level != LogLevel::Invalid)
    {
        Logging::setEnabled(level, enable);
        return;
    }

    auto component = Logging::stringToComponent(str);
    if(component != Logging::Component::Invalid)
    {
        Logging::setEnabled(component, enable);
        return;
    }
}

int main(int argc, char *argv[])
{
    int screenWidth = 240;
    int screenHeight = 240;
    int screenScale = 5;

    std::thread gdbServerThread;

    bool usbEnabled = false;
    bool gdbEnabled = false;

    std::string romFilename;

    // disable some log levels by default
    Logging::setEnabled(LogLevel::Debug, false);
    Logging::setEnabled(LogLevel::NotImplemented, false); // very noisy (for now?)

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else if(arg == "--picosystem-sdk")
            picosystemSDK = true;
        else if(arg == "--board" && i + 1 < argc)
            board = stringToBoard(argv[++i]);
        else if(arg == "--usb")
            usbEnabled = true;
        else if(arg == "--usbip")
        {
            usbEnabled = true;
            mem.getUSB().setUSBIPEnabled(true);
        }
        else if(arg == "--log" && i + 1 < argc)
            handleLogArg(argv[++i]);
        else if(arg == "--gdb")
            gdbEnabled = true;
        else if(arg == "--iolog" && i + 1 < argc)
            mem.getGPIO().openLogFile(argv[++i]);
        else
            break;
    }

    if(i == argc)
        logf(LogLevel::Info, logComponent, "No file specified!");
    else
        romFilename = argv[i];

    // get base path
    std::string basePath;
    auto tmp = SDL_GetBasePath();
    if(tmp)
    {
        basePath = tmp;
        SDL_free(tmp);
    }

    if(!romFilename.empty())
    {
        uf2File.open(romFilename, std::ifstream::in | std::ifstream::binary);

        if(!uf2File || !parseUF2(uf2File))
        {
            logf(LogLevel::Error, logComponent, "Failed to open UF2 \"%s\"", romFilename.c_str());
            return 1;
        }

        // picosystem SDK does not require the correct board to be set... so most uf2s don't
        if(picosystemSDK)
            board = Board::PimoroniPicoSystem;
    }

    // default board if still not set
    if(board == Board::Unknown)
    {
        logf(LogLevel::Info, logComponent, "Board not specified, falling back to \"pico\""); 
        board = Board::Pico;
    }

    getBoardScreenSize(board, screenWidth, screenHeight);

    // emu init
    mem.setCPUs(cpuCores);

    std::ifstream bootROMFile(basePath + "bootrom.bin", std::ifstream::in | std::ifstream::binary);
    if(bootROMFile)
    {
        bootROMFile.read(reinterpret_cast<char *>(bootROM), sizeof(bootROM));

        mem.setBootROM(bootROM);
    }
    else
    {
        logf(LogLevel::Error, logComponent, "bootrom.bin not found!");
        return 1;
    }

    auto &clocks = mem.getClocks();

    for(auto &core : cpuCores)
        clocks.addClockTarget(5, core.getClock());

    mem.reset();

    // external hardware
    if(board == Board::PimoroniPicoSystem)
    {
        mem.setInterruptUpdateCallback(onInterruptUpdate);
        mem.setGetNextInterruptTimeCallback(onGetNextInterruptTime);

        mem.getGPIO().setReadCallback(onGPIORead);
        mem.getGPIO().clearInputFloatingMask(1 << 8); // TE

        mem.getPIO(0).setUpdateCallback(onPIOUpdate);

        mem.getPWM().setOutputCallback(onPWMUpdate, 1 << 11); // audio

        const int fps = picosystemSDK ? 40 : 50;
        displayClock.setFrequency(fps * 240);
        clocks.addClockTarget(-1, displayClock);

        audioClock.setFrequency(48000);
        clocks.addClockTarget(-1, audioClock);
    }

    if(gdbEnabled)
        gdbServerThread = std::thread(runGDBServer);

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        logf(LogLevel::Error, logComponent, "Failed to init SDL!");
        return 1;
    }

    bool boardHasScreen = screenWidth && screenHeight;
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    SDL_Texture *texture = nullptr;

    if(boardHasScreen)
    {
        window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                    screenWidth * screenScale, screenHeight * screenScale,
                                    SDL_WINDOW_RESIZABLE);

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
        SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
        SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

        Uint32 format;

        // TODO: implement screen registers instead of guessing
        if(picosystemSDK)
            format = SDL_PIXELFORMAT_ARGB4444;
        else if(board == Board::PimoroniPicoSystem) // assume 32blit-sdk
            format = SDL_PIXELFORMAT_BGR565;
        else
            format = SDL_PIXELFORMAT_RGB565;

        texture = SDL_CreateTexture(renderer, format, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);
    }

    SDL_AudioDeviceID audioDevice = 0;

    if(board == Board::PimoroniPicoSystem)
    {
        SDL_AudioSpec spec{};

        spec.freq = 48000;
        spec.format = AUDIO_S16;
        spec.channels = 1;
        spec.samples = 512;
        spec.callback = audioCallback;

        audioDevice = SDL_OpenAudioDevice(nullptr, false, &spec, nullptr, 0);

        if(!audioDevice)
        {
            logf(LogLevel::Error, logComponent, "Failed to open audio: %s", SDL_GetError());
            quit = true;
        }

        SDL_PauseAudioDevice(audioDevice, 0);
    }

    auto lastTick = SDL_GetTicks();

    auto lastFreqUpdate = lastTick;
    uint32_t cpuCycles = 0;

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        // update freq info
        if(now - lastFreqUpdate >= 1000)
        {
            if(window)
            {
                auto time = now - lastFreqUpdate;
                int speedPercent = static_cast<uint64_t>(cpuCycles) * 1000 / time * 100 / clocks.getClockFrequency(5);
                char buf[50];
            
                snprintf(buf, sizeof(buf), "DERP | SYS: %iMHz (%i%%)", cpuCycles / (1000 * time), speedPercent);
                SDL_SetWindowTitle(window, buf);
            }

            cpuCycles = 0;
            lastFreqUpdate = now;
        }

        auto elapsed = now - lastTick;

        // clamp if running behind
        if(elapsed > 30)
            elapsed = 30;

        // lock cpu access around updates
        if(gdbEnabled)
            gdbServer.getCPUMutex().unlock();

        cpuCycles += cpuCores[0].run(elapsed);

        // sync core 1
        // TODO: more sync
        auto time = cpuCores[0].getClock().getTime();
        cpuCores[1].update(time);

        // sync peripherals
        mem.peripheralUpdate(time);

        if(gdbEnabled)
            gdbServer.getCPUMutex().unlock();

        if(board == Board::PimoroniPicoSystem)
        {
            displayUpdate(time);
            audioUpdate(time);
        }

        // attempt to connect USB
        if(usbEnabled)
        {
            auto &usb = mem.getUSB();
            if(usb.getEnabled() && !usb.getConfigured())
                usb.startEnumeration();

            usb.usbipUpdate();
        }

        // adjust timers to stay in range
        clocks.adjustClocks();

        lastTick = now;

        if(renderer)
        {
            // TODO: sync
            SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, nullptr, nullptr);
            SDL_RenderPresent(renderer);
        }
    }

    if(texture)
        SDL_DestroyTexture(texture);
    if(renderer)
        SDL_DestroyRenderer(renderer);
    if(window)
        SDL_DestroyWindow(window);

    if(audioDevice)
        SDL_CloseAudioDevice(audioDevice);

    if(gdbEnabled)
        gdbServerThread.join();
    return 0;
}
