#include <iostream>
#include <string>

#include "parse_args.hpp"
#include "parse_rom.hpp"
#include "mappers.hpp"
#include "game_loop.hpp"

int main(int argc, char* argv[]){

    // Parse the arguments provided by the user
    struct nes_args args = parse_nes_args(argc, argv);

    // Parse game information from file, load ROM into memory
    Cartridge* game_cartridge = load_rom(args.filename);

    // NOTE: Will not return until the emulator is exited
    game_loop(game_cartridge);
    free(game_cartridge);
}

