#include <fstream>
#include <iostream>
#include <unordered_map>

#include <SDL.h>

#include "ARMv6MCore.h"

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpu(mem);

static uint8_t bootROM[0x4000];

static std::ifstream uf2File;

uint16_t screenData[240 * 240];

static uint32_t buttonState = 0;

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

        auto ptr = cpu.getMem().mapAddress(block.addr);
        if(ptr)
            memcpy(ptr, block.payload, block.payloadSize);
        else
            std::cerr << "Can't write UF2 payload to " << std::hex << block.addr << std::dec << "!\n";
    }

    return true;
}

int main(int argc, char *argv[])
{
    int screenWidth = 240;
    int screenHeight = 240;
    int screenScale = 5;

    std::string romFilename;

    int i = 1;

    for(; i < argc; i++)
    {
        std::string arg(argv[i]);

        if(arg == "--scale" && i + 1 < argc)
            screenScale = std::stoi(argv[++i]);
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
    }

    // emu init
    mem.setCPU(&cpu);

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

    clocks.addClockTarget(5, cpu.getClock());

    cpu.reset();

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

    auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR565, SDL_TEXTUREACCESS_STREAMING, screenWidth, screenHeight);

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

        // picosystem IO
        auto &gpio = mem.getGPIO();

        auto inputs = gpio.getInputs();

        // update buttons
        int buttonMask = 0xFF0000;
        inputs = (inputs & ~buttonMask) | (buttonState ^ buttonMask);

        // toggle TE
        // FIXME: correct timings
        gpio.setInputs(inputs & ~(1 << 8));
        cpuCycles += cpu.run(1); // picosystem sdk wants to see the transition
        elapsed--;
        gpio.setInputs(inputs | (1 << 8));

        cpuCycles += cpu.run(elapsed);
        // sync peripherals
        mem.peripheralUpdate(cpu.getClock().getTime());

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
