#include "game_loop.hpp"
#include "cpu.hpp"
#include "ppu.hpp"

void game_loop (Cartridge* game_cartridge) {

	Init_CPU(game_cartridge);
	Init_PPU(game_cartridge);

	while (1) {
		cycle_cpu();
		cycle_ppu();
		cycle_ppu();
		cycle_ppu();
	}
}
