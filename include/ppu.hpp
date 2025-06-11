#ifndef PPU_H
#define PPU_H
#include "mappers.hpp"
#include <cstdint>

// Functions for game loop to use
void Init_PPU (Cartridge* game_cartridge);
void cycle_ppu ();

// Access to CPU memory-mapped PPU register
void write_ppu_from_cpu (uint8_t addr, uint8_t data);
uint8_t read_ppu_from_cpu (uint8_t addr);

#endif
