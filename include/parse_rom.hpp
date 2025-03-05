#ifndef PARSE_ROM_H
#define PARSE_ROM_H

#include <string>

enum nes_tv_system {NTSC, PAL, MULTIREGION, DENDY};
enum nes_console_type {NES, VS_SYSTEM, PLAYCHOICE_10, EXTENDED};
enum vs_system {RP2C03, RP2C04_0001, RP2C04_0002, RP2C04_0003, RP2C04_0004,
				RP2C05_01, RP2C05_02, RP2C05_03, RP2C05_04};
enum nes_nametable_arrangement {VERT, HORIZ};
enum expansion_device {UNSPECIFIED, NES_CONTROLLER};

struct ines_info {
	uint16_t prg_rom_size = 0;
	uint16_t chr_rom_size = 0;

	uint16_t mapper = 0;

	nes_nametable_arrangement nametable_arrangement;
	bool persistent_memory = false;
	bool trainer = false;
	bool alt_nametable_layout = false;
	nes_console_type console_type;
	bool ines_2 = false;

	uint32_t prg_ram_size = 0;

	nes_tv_system tv_system;
	bool vs_system = false;
	uint8_t vs_system_type;


	// iNES 2.0 specific fields
	uint32_t chr_ram_size = 0;
	uint32_t prg_nvram_size = 0;
	uint32_t chr_nvram_size = 0;

	uint8_t submapper = 0;

	uint8_t vs_hardware_type;
	uint8_t extended_console_type;

	uint8_t misc_roms = 0;;
	expansion_device default_expansion_dev;

	// iNES 1 specific fields
	bool prg_ram_present = false;
	bool bus_conflicts = false;

};

struct ines_header {
	uint32_t label;
	uint8_t prg_size;
	uint8_t chr_size;
	uint8_t flags_6;
	uint8_t flags_7;
	uint8_t flags_8;
	uint8_t flags_9;
	uint8_t flags_10;
	uint8_t flags_11;
	uint8_t flags_12;
	uint8_t flags_13;
	uint8_t flags_14;
	uint8_t flags_15;
};

struct ines_info parse_rom (std::string filename);

#endif
