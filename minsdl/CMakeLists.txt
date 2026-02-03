# minimal SDL shell

add_executable(DERP_SDL
    Main.cpp
    board/Interstate75.cpp
    board/PicoSystem.cpp
    board/PicoVision.cpp
    board/Tufty2040.cpp
)

find_package(SDL2 REQUIRED)

target_link_libraries(DERP_SDL DERPCore usbip-server SDL2::SDL2)

if(SDL2_SDL2main_FOUND)
    target_link_libraries(DERP_SDL SDL2::SDL2main)
endif()

install(TARGETS DERP_SDL
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION bin
)

