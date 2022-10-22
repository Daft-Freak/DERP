#include <fstream>
#include <iostream>

#include <SDL.h>

#include "ARMv6MCore.h"

static bool quit = false;

static MemoryBus mem;
static ARMv6MCore cpu(mem);

static uint8_t bootROM[0x4000];

static std::ifstream uf2File;

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

static void pollEvents()
{
    SDL_Event event;
    while(SDL_PollEvent(&event))
    {
        switch(event.type)
        {
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
    int screenWidth = 160;
    int screenHeight = 144;
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
        uf2File.open(romFilename);

        if(!uf2File || !parseUF2(uf2File))
        {
            std::cerr << "Failed to open UF2 \"" << romFilename << "\"\n";
            return 1;
        }
    }

    // emu init
    auto &mem = cpu.getMem();

    std::ifstream bootROMFile(basePath + "bootrom.bin");
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

    cpu.reset();

    auto &clocks = mem.getClocks();
   
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
    uint32_t skippedTime = 0;

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        // update freq info
        if(now - lastFreqUpdate >= 1000)
        {
            auto time = now - lastFreqUpdate;
            int speedPercent = time * 100 / (time + skippedTime);
            char buf[50];

            snprintf(buf, sizeof(buf), "DERP | SYS: %iMHz (%i%%)", cpuCycles / (1000 * time), speedPercent);
            SDL_SetWindowTitle(window, buf);

            cpuCycles = 0;
            lastFreqUpdate = now;
            skippedTime = 0;
        }

        auto elapsed = now - lastTick;

        // clamp if running behind
        if(elapsed > 30)
        {
            skippedTime += elapsed - 30;
            elapsed = 30;
        }

        cpu.getClock().setClockScale(clocks.getClockScale(5)); // FIXME: need to set this when the clock changes
        cpuCycles += cpu.run(elapsed);

        // adjust timers to stay in range
        // ... which is easy becuse there's only one
        auto emuTime = cpu.getClock().getTime();
        if(emuTime & (1ull << 63))
            cpu.getClock().adjustTime(emuTime);

        lastTick = now;

        // TODO: sync
        //SDL_UpdateTexture(texture, nullptr, screenData, screenWidth * 2);
        SDL_RenderClear(renderer);
        //SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    return 0;
}
