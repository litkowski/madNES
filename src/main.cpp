#include <iostream>
#include <string>

#include "parse_args.hpp"
#include "parse_rom.hpp"

int main(int argc, char* argv[]){

    // Parse the arguments provided by the user
    struct nes_args args = parse_nes_args(argc, argv);

    std::cout << "NES ROM provided: "  + args.filename + "\n";
    std::cout << "Disassemble ROM? " + std::to_string(args.dump_rom) + " UNSUPPORTED \n";

    struct ines_info rom_info = parse_rom(args.filename);
    return 0;
}

