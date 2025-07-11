#include "game_loop.hpp"
#include "graphics.hpp"
#include "cpu.hpp"
#include "ppu.hpp"

void game_loop (Cartridge* game_cartridge) {

	Init_CPU(game_cartridge);
	Init_PPU(game_cartridge);
	Init_Graphics_And_IO();
	Init_Master_Palette("./palette.pal");

	/* while (1) {
		cycle_cpu();
		cycle_ppu();
		cycle_ppu();
		cycle_ppu();
	} */

	ppu_game_loop();
}
