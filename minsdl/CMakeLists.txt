# minimal SDL shell

add_executable(DERP_SDL Main.cpp)

find_package(SDL2 REQUIRED)

target_link_libraries(DERP_SDL DERPCore SDL2::SDL2)
