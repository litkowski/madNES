#ifndef MAPPER_H
#define MAPPER_H

#include <cstdint>
#include <vector>

#include "parse_rom.hpp"

class Cartridge {
	protected:
		std::vector<uint8_t> cpu_memory;
		std::vector<uint8_t> ppu_memory;
	public:
		virtual uint8_t cpu_read (uint16_t address) = 0;
		virtual uint8_t ppu_read (uint16_t address) = 0;
		virtual void cpu_write (uint16_t address, uint8_t data) = 0;
		virtual void ppu_write (uint16_t address, uint8_t data) = 0;
};

class NROM : public Cartridge {
	private:
		uint16_t prg_ram_size;
		uint16_t prg_rom_size;
		uint16_t chr_rom_size;
		nes_nametable_arrangement nametable_arrangement;
	public:
		NROM (struct ines_info, std::string filename);
		uint8_t cpu_read (uint16_t address);
		uint8_t ppu_read (uint16_t address);
		void cpu_write (uint16_t address, uint8_t data);
		void ppu_write (uint16_t address, uint8_t data);

};

Cartridge* load_rom (std::string filename);

#endif
