#include <iostream>
#include <string>
#include <signal.h>

#include "parse_args.hpp"
#include "parse_rom.hpp"
#include "mappers.hpp"
#include "game_loop.hpp"

void signal_handler (int x) {
    exit(EXIT_SUCCESS);
}

int main(int argc, char* argv[]){

    // Ensure we abort on a SIGINT (ctrl-c)
    signal(SIGINT, &signal_handler);

    // Parse the arguments provided by the user
    struct nes_args args = parse_nes_args(argc, argv);

    // Parse game information from file, load ROM into memory
    Cartridge* game_cartridge = load_rom(args.filename);

    // NOTE: Will not return until the emulator is exited
    game_loop(game_cartridge);
    free(game_cartridge);
}

