#include <filesystem>
#include <fstream>

#include "parse_rom.hpp"

// Parse the given ROM's header according to the iNES format.
// NOTE: Only the .nes (iNES) format is currently supported, both iNES and iNES 2.0
struct ines_info parse_rom (std::string filename) {

	// Ensure the file is of type .nes
	std::filesystem::path file_path = filename;

	if (file_path.extension() != ".nes") {
		printf("Currently only .nes (ines) files are supported.\n");
		throw std::runtime_error("Given ROM" + filename + "is not of type .nes.");
	}

	// Open the ROM's filestream, read in its header, and close it
	std::ifstream rom;
	rom.open(filename, std::fstream::in);
	struct ines_header header;
	rom.read((char*) &header, 16);
	rom.close();

	// Ensure that the iNES label is formatted correctly
	if (header.label != 0x1A53454E) {
		printf("ROM label formatted incorrectly.\n");
		throw std::runtime_error("ROM label formatted incorrectly.");
	}

	// Extract PRG and CHR ROM sizes
	struct ines_info rom_info;
	rom_info.prg_rom_size = 0x4000 * header.prg_size;
	rom_info.chr_rom_size = 0x2000 * header.chr_size;

	// Extract mapper index lower bytes
	rom_info.mapper = (0xF0 & header.flags_6) >> 4;

	// Extract nametable arrangement
	if (0b1 & header.flags_6 == 0) {
		rom_info.nametable_arrangement = HORIZ;
	} else {
		rom_info.nametable_arrangement = VERT;
	}

	// Extract bools common to iNES and iNES 2.0
	rom_info.persistent_memory = (0b10 & header.flags_6);
	rom_info.trainer = (0b100 & header.flags_6);
	rom_info.alt_nametable_layout = (0b1000 & header.flags_6);

	// Extract the NES console type
	switch (header.flags_7 & 0b11) {
		case (0):
			rom_info.console_type = NES;
			break;
		case (1):
			rom_info.console_type = VS_SYSTEM;
			break;
		case (2):
			rom_info.console_type = PLAYCHOICE_10;
			break;
		case (3):
			rom_info.console_type = EXTENDED;
			break;
	}

	// Extract iNES 2.0 information and upper byte of mapper
	rom_info.ines_2 = (0b1100 & header.flags_7) == 0b1000;
	rom_info.mapper |= (0x0F & header.flags_7);

	if (rom_info.ines_2) {

		// Parse iNES 2.0 format

		// Extract mapper highest byte and submapper information
		rom_info.mapper |= ((0b1111 & header.flags_8) << 8);
		rom_info.submapper = (0b11110000 & header.flags_8) >> 4;

		// Extract highest byte of PRG and CHR ROM
		rom_info.prg_rom_size |= (0b1111 & header.flags_9) << 8;
		rom_info.chr_rom_size |= (0b11110000 & header.flags_9) << 4;

		// Extract CHR and PRG RAM sizes
		if (header.flags_10 & 0b1111) {
			rom_info.prg_ram_size = 64 << (header.flags_10 & 0b1111);
		} else {
			rom_info.prg_ram_size = 0;
		}

		if (header.flags_10 & 0b11110000) {
			rom_info.prg_nvram_size = 64 << ((header.flags_10 & 0b11110000) >> 4);
		} else {
			rom_info.prg_nvram_size = 0;
		}

		if (header.flags_11 & 0b1111) {
			rom_info.chr_ram_size = 64 << (header.flags_11 & 0b1111);
		} else {
			rom_info.chr_ram_size = 0;
		}

		if (header.flags_11 & 0b11110000) {
			rom_info.chr_nvram_size = 64 << ((header.flags_11 & 0b11110000) >> 4);
		} else {
			rom_info.chr_nvram_size = 0;
		}

		// Extract NES TV system type
		switch (header.flags_12 & 0b11) {
			case (0):
				rom_info.tv_system = NTSC;
				break;
			case (1):
				rom_info.tv_system = PAL;
				break;
			case (2):
				rom_info.tv_system = MULTIREGION;
				break;
			case (3):
				rom_info.tv_system = DENDY;
				break;
		}

		// Extract Vs. System type, if it exists
		if (rom_info.console_type == VS_SYSTEM) {

			switch (header.flags_13 & 0b1111) {
				case (0):
					rom_info.vs_system_type = RP2C03;
					break;
				case (2):
					rom_info.vs_system_type = RP2C04_0001;
					break;
				case (3):
					rom_info.vs_system_type = RP2C04_0002;
					break;
				case (4):
					rom_info.vs_system_type = RP2C04_0003;
					break;
				case (5):
					rom_info.vs_system_type = RP2C04_0004;
					break;
				case (8):
					rom_info.vs_system_type = RP2C05_01;
					break;
				case (9):
					rom_info.vs_system_type = RP2C05_02;
					break;
				case (10):
					rom_info.vs_system_type = RP2C05_03;
					break;
				case (11):
					rom_info.vs_system_type = RP2C05_04;
					break;
				default:
					throw std::runtime_error("Invalid Vs. System type");
			}

		} else if (rom_info.console_type == EXTENDED) {
			// TODO: self-explanatory
			throw std::runtime_error("Extended console types not currently supported");
		}

		// Extract the default expansion device
		switch (header.flags_15) {
			case (0):
				rom_info.default_expansion_dev = UNSPECIFIED;
				break;
			case (1):
				rom_info.default_expansion_dev = NES_CONTROLLER;
				break;
			default:
				throw std::runtime_error("Only NES controllers currently supported");
		}

	} else {

		// Parse standard iNES format

		// Extract PRG RAM size
		rom_info.prg_ram_size = header.flags_8;

		// Extract header TV system
		if (header.flags_9 == 0) {
			rom_info.tv_system = NTSC;
		} else if (header.flags_9 == 1) {
			rom_info.tv_system = PAL;
		}

		if (header.flags_10 & 0b11 == 0) {
			rom_info.tv_system = NTSC;
		} else if (header.flags_10 & 0b11 == 0b10) {
			rom_info.tv_system = PAL;
		} else {
			rom_info.tv_system = MULTIREGION;
		}

		// Parse the final flags found in byte 10
		rom_info.prg_ram_present =  (header.flags_10 & 0b00010000);
		rom_info.bus_conflicts = (header.flags_10 & 0b0010000);
	}

	return rom_info;
}
