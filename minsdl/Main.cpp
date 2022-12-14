#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

#include <SDL.h>

#include "ARMv6MCore.h"

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpuCores[2]{mem, mem};

static uint8_t bootROM[0x4000];

static std::ifstream uf2File;

// TODO: maybe an enum if other boards get supported
static bool isPicoSystem = false;

uint16_t screenData[240 * 240];

static uint32_t buttonState = 0;
static unsigned int displayScanline = 0;
static ClockTarget displayClock;

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

static bool parseUF2(std::ifstream &file)
{
    while(!file.eof())
    {
        UF2Block block;
        file.read(reinterpret_cast<char *>(&block), sizeof(block));

        if(block.magicStart[0] != uf2MagicStart0 || block.magicStart[1] != uf2MagicStart1 || block.magicEnd != uf2MagicEnd)
        {
            std::cerr << "Bad UF2 block magic!\n";
            return false;
        }

        auto ptr = mem.mapAddress(block.addr);
        if(ptr)
            memcpy(ptr, block.payload, block.payloadSize);
        else
            std::cerr << "Can't write UF2 payload to " << std::hex << block.addr << std::dec << "!\n";
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
        std::cout << "binary_info:\n";

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
                    std::cout << "\t" << idStr->second << ": " << str << "\n";

                // detect board
                if(id == 0xb63cffbb/*pico_board*/ && std::string_view(str) == "pimoroni_picosystem")
                    isPicoSystem = true;
            }
        }

        std::cout << "\n";
    }

    return true;
}

// picosystem external hardware/IO
static void displayUpdate(uint64_t time)
{
    auto lines = displayClock.getCyclesToTime(time);

    if(!lines)
        return;

    while(lines)
    {
        int step = std::max(1u, std::min(lines, 239 - displayScanline));
        lines -= step;

        // set TE when we're on the last scanline
        // TODO: use STE reg
        auto newLine = (displayScanline + step) % 240;

        if(newLine == 239) // now last line
            mem.getGPIO().setInputMask(1 << 8);
        else if(displayScanline == 239) // was last line
            mem.getGPIO().clearInputMask(1 << 8);

        displayScanline = newLine;
        displayClock.addCycles(step);
    }
}

static uint32_t onGPIORead(uint64_t time, uint32_t inputs)
{
    // apply buttons
    int buttonMask = 0xFF0000;
    inputs = (inputs & ~buttonMask) | (buttonState ^ buttonMask);

    displayUpdate(time);

    return inputs;
}

static void onInterruptUpdate(uint64_t time, uint32_t irqMask)
{
    if(irqMask & (1 << 13) /*IO_IRQ_BANK0*/)
        displayUpdate(time);
}

static uint64_t onGetNextInterruptTime(uint64_t time)
{
    if(!mem.getGPIO().interruptsEnabledOnPin(8))
        return time;

    int lines = std::max(1u, 239 - displayScanline);
    auto ret = displayClock.getTimeToCycles(lines);

    return ret;
}

int main(int argc, char *argv[])
{
    int screenWidth = 240;
    int screenHeight = 240;
    int screenScale = 5;

    bool picosystemSDK = false;

    std::string romFilename;

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
        else if(arg == "--picosystem-sdk")
            picosystemSDK = true;
        else
            break;
    }

    if(i == argc)
        std::cout << "No ROM specified!\n";
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
            std::cerr << "Failed to open UF2 \"" << romFilename << "\"\n";
            return 1;
        }

        // picosystem SDK does not require the correct board to be set... so most uf2s don't
        if(picosystemSDK)
            isPicoSystem = true;
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
        std::cerr << "bootrom.bin not found \n";
        return 1;
    }

    auto &clocks = mem.getClocks();

    for(auto &core : cpuCores)
    {
        clocks.addClockTarget(5, core.getClock());

        core.reset();
    }

    // external hardware
    if(isPicoSystem)
    {
        mem.setInterruptUpdateCallback(onInterruptUpdate);
        mem.setGetNextInterruptTimeCallback(onGetNextInterruptTime);
        mem.getGPIO().setReadCallback(onGPIORead);

        const int fps = picosystemSDK ? 40 : 50;
        displayClock.setFrequency(fps * 240);
        clocks.addClockTarget(-1, displayClock);
    }

    // SDL init
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        std::cerr << "Failed to init SDL!\n";
        return 1;
    }

    auto window = SDL_CreateWindow("DaftBoySDL", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                   screenWidth * screenScale, screenHeight * screenScale,
                                   SDL_WINDOW_RESIZABLE);

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
    SDL_RenderSetLogicalSize(renderer, screenWidth, screenHeight);
    SDL_RenderSetIntegerScale(renderer, SDL_TRUE);

    auto texture = SDL_CreateTexture(renderer, picosystemSDK ? SDL_PIXELFORMAT_ARGB4444 : SDL_PIXELFORMAT_BGR565, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

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
            auto time = now - lastFreqUpdate;
            int speedPercent = static_cast<uint64_t>(cpuCycles) * 1000 / time * 100 / clocks.getClockFrequency(5);
            char buf[50];

            snprintf(buf, sizeof(buf), "DERP | SYS: %iMHz (%i%%)", cpuCycles / (1000 * time), speedPercent);
            SDL_SetWindowTitle(window, buf);

            cpuCycles = 0;
            lastFreqUpdate = now;
        }

        auto elapsed = now - lastTick;

        // clamp if running behind
        if(elapsed > 30)
            elapsed = 30;

        cpuCycles += cpuCores[0].run(elapsed);

        // sync core 1
        // TODO: more sync
        auto time = cpuCores[0].getClock().getTime();
        cpuCores[1].update(time);

        // sync peripherals
        mem.peripheralUpdate(time);

        if(isPicoSystem)
            displayUpdate(time);

        // adjust timers to stay in range
        clocks.adjustClocks();

        lastTick = now;

        // TODO: sync
        SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
