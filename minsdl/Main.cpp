#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include <SDL.h>

#include "ARMv6MCore.h"
#include "GDBServer.h"
#include "Logging.h"

#include "board/Pico.h"
#include "board/Interstate75.h"
#include "board/PicoSystem.h"
#include "board/PicoVision.h"
#include "board/Tufty2040.h"

using Logging::logf;
using LogLevel = Logging::Level;
constexpr auto logComponent = Logging::Component::Main;

enum class BoardId
{
    Unknown = -1,
    Pico = 0,

    PimoroniInterstate75,
    PimoroniPicoSystem,
    PimoroniPicoVision,
    PimoroniTufty2040,
};

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpuCores[2]{mem, mem};

static GDBServer gdbServer;

static uint8_t bootROM[0x4000];

static std::ifstream uf2File;

static BoardId boardId = BoardId::Unknown;
static Board *board = nullptr;

static SDL_Renderer *renderer = nullptr;
static SDL_Texture *texture = nullptr;
static int screenWidth = 240;
static int screenHeight = 240;

static SDL_AudioDeviceID audioDevice = 0;

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


static BoardId stringToBoard(std::string_view str)
{
    if(str == "pico")
        return BoardId::Pico;

    if(str == "pimoroni_interstate75")
        return BoardId::PimoroniInterstate75;

    if(str == "pimoroni_picosystem")
        return BoardId::PimoroniPicoSystem;

    if(str == "pimoroni_picovision")
        return BoardId::PimoroniPicoVision;

    if(str == "pimoroni_tufty2040")
        return BoardId::PimoroniTufty2040;

    logf(LogLevel::Warning, logComponent, "Unknown board \"%.*s\", falling back to \"pico\"", int(str.length()), str.data()); 
    return BoardId::Pico;
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
                if(id == 0xb63cffbb/*pico_board*/ && boardId == BoardId::Unknown)
                    boardId = stringToBoard(str);
            }
        }
    }

    return true;
}

static void runGDBServer()
{
    gdbServer.setCPUs(cpuCores, 2);

    if(!gdbServer.start()) {
        logf(LogLevel::Error, logComponent, "Failed to start GDB server!");
        return;
    }

    while(!quit)
    {
        if(!gdbServer.update(true))
           logf(LogLevel::Error, logComponent, "Failed to update GDB server!");
    }
}

static void pollEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        board->handleEvent(event);

        switch(event.type)
        {
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

void updateScreenSettings()
{
    if(!renderer)
        return;

    // check for size change
    int newWidth, newHeight;
    board->getScreenSize(newWidth, newHeight);

    if(newWidth != screenWidth || newHeight != screenHeight)
    {
        screenWidth = newWidth;
        screenHeight = newHeight;

        // update renderer
        SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
        SDL_RenderSetIntegerScale(renderer, SDL_TRUE);
    }

    texture = SDL_CreateTexture(renderer, board->getScreenFormat(), SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);
}

void queueAudio(const int16_t *samples, unsigned int numSamples)
{
    SDL_QueueAudio(audioDevice, samples, numSamples * sizeof(int16_t));
}

int main(int argc, char *argv[])
{
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
        {
            // picosystem SDK does not require the correct board to be set... so most uf2s don't
            boardId = BoardId::PimoroniPicoSystem;
            logf(LogLevel::Warning, logComponent, "Use --board pimoroni_picosystem instead of --picosystem-sdk");
        }
        else if(arg == "--board" && i + 1 < argc)
            boardId = stringToBoard(argv[++i]);
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
    }

    // default board if still not set
    if(boardId == BoardId::Unknown)
    {
        logf(LogLevel::Info, logComponent, "Board not specified, falling back to \"pico\""); 
        boardId = BoardId::Pico;
    }

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

    // create board
    switch(boardId)
    {
        case BoardId::PimoroniInterstate75:
            board = new Interstate75Board(mem);
            break;

        case BoardId::PimoroniPicoSystem:
            board = new PicoSystemBoard(mem);
            break;

        case BoardId::PimoroniPicoVision:
            board = new PicoVisionBoard(mem);
            break;

        case BoardId::PimoroniTufty2040:
            board = new Tufty2040Board(mem);
            break;

        default:
            board = new PicoBoard(mem);
    }

    board->getScreenSize(screenWidth, screenHeight);

    if(gdbEnabled)
        gdbServerThread = std::thread(runGDBServer);

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        logf(LogLevel::Error, logComponent, "Failed to init SDL!");
        return 1;
    }

    bool boardHasScreen = screenWidth && screenHeight;
    int boardPixelStride = 2;
    SDL_Window *window = nullptr;

    if(boardHasScreen)
    {
        window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                    screenWidth * screenScale, screenHeight * screenScale,
                                    SDL_WINDOW_RESIZABLE);

        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
        SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
        SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

        texture = SDL_CreateTexture(renderer, board->getScreenFormat(), SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

        // get format info
        auto format = SDL_AllocFormat(board->getScreenFormat());
        if(format)
        {
            boardPixelStride = format->BitsPerPixel / 8;
            SDL_FreeFormat(format);
        }
    }

    if(board->hasAudio())
    {
        SDL_AudioSpec spec{};

        spec.freq = 48000;
        spec.format = AUDIO_S16;
        spec.channels = 1;
        spec.samples = 512;

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
            gdbServer.getCPUMutex().lock();

        cpuCycles += cpuCores[0].run(elapsed);

        // sync core 1
        // TODO: more sync
        auto time = cpuCores[0].getClock().getTime();
        cpuCores[1].update(time);

        // sync peripherals
        mem.peripheralUpdate(time);

        if(gdbEnabled)
            gdbServer.getCPUMutex().unlock();

        board->update(time);

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
            SDL_UpdateTexture(texture, nullptr, board->getScreenData(), screenWidth * boardPixelStride);
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

    delete board;

    if(gdbEnabled)
        gdbServerThread.join();
    return 0;
}
