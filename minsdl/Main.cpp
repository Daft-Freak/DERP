#include <fstream>
#include <iostream>

#include <SDL.h>

#include "ARMv6MCore.h"

static bool quit = false;

static ARMv6MCore cpu;

static uint8_t bootROM[0x4000];

static std::ifstream romFile;

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
        romFile.open(romFilename);

        if(!romFile)
        {
            std::cerr << "Failed to open ROM \"" << romFilename << "\"\n";
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

    while(!quit)
    {
        pollEvents();

        auto now = SDL_GetTicks();

        cpu.run(now - lastTick);

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
