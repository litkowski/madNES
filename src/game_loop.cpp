#include "game_loop.hpp"
#include "cpu.hpp"

void game_loop (Cartridge* game_cartridge) {

	CPU cpu = CPU(game_cartridge);

	cpu.cycle();
	cpu.cycle();
}
