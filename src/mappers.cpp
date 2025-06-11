#include <stdexcept>
#include <fstream>

#include "mappers.hpp"
#include "ppu.hpp"

// Construct an NROM Cartridge object using iNES info
NROM::NROM (struct ines_info rom_info, std::string filename) {

    // Initialize and zero out memory
    cpu_memory = std::vector<uint8_t>(0x10000, 0);
    ppu_memory = std::vector<uint8_t>(0x2000, 0);

    // Copy PRG RAM and ROM sizes, as well as CHR RAM size
    prg_ram_size = rom_info.prg_ram_size;
    if (prg_ram_size > 0x2000) {
        throw std::runtime_error("NROM standard can only handle 8kB of PRG-RAM");
    }

    prg_rom_size = rom_info.prg_rom_size;
    if (prg_rom_size > 0x8000) {
        throw std::runtime_error("NROM standard can only handle 32kB of PRG-ROM");
    }

    chr_rom_size = rom_info.chr_rom_size;
    if (chr_rom_size > 0x2000) {
        throw std::runtime_error("NROM standard can only handle 8kB of CHR-ROM");
    }

    // Record nametable mirroring setup
    nametable_arrangement = rom_info.nametable_arrangement;

    // Read supported fields from the iNES format
    std::ifstream rom;
   	rom.open(filename, std::fstream::in);
    rom.seekg(16);

    if (rom_info.trainer) {
        throw std::runtime_error("NROM standard does not support trainer");
    } else {
        rom.read((char*) cpu_memory.data() + 0x8000, prg_rom_size);
        rom.read((char*) ppu_memory.data(), chr_rom_size);
    }

    rom.close();
}

// Map CPU addresses onto physical addresses for reading purposes
// Emulates default NES address mirroring
uint8_t NROM::cpu_read (uint16_t addr) {

    if (addr <= 0x07FF) {
        return cpu_memory[addr];
    } else if (addr <= 0x1FFF) {
        return cpu_memory[addr % 0x0800];
    } else if (addr <= 0x2007) {
        return read_ppu_from_cpu(addr % 0x0008);
    } else if (addr == 0x4014) {
        return read_ppu_from_cpu(addr % 16);
    } else if (addr <= 0x4017) {
        return cpu_memory[addr];
    } else if (addr <= 0x401F) {
        return cpu_memory[addr];
    } else if (addr <= 0x5FFF) {
        return 0;
    } else if (addr <= 0x7FFF) {
        return cpu_memory[addr];
    } else if (addr <= 0xFFFF) {
        return cpu_memory[0x8000 + ((addr - 0x8000) % prg_rom_size)];
    }

}

// Read from PPU memory,
uint8_t NROM::ppu_read (uint16_t addr) {

    if (addr <= 0x0FFF) {
        return ppu_memory[addr];
    } else if (addr <= 0x23FF) {
        return ppu_memory[addr];
    } else if (addr <= 0x27FF) {

        if (nametable_arrangement == VERT) {
            return ppu_memory[addr];
        } else {
            return ppu_memory[0x2000 + ((addr - 0x2400) % 0x0400)];
        }

    } else if (addr <= 0x2BFF) {

        if (nametable_arrangement == HORIZ) {
            return ppu_memory[addr];
        } else {
            return ppu_memory[0x2000 + ((addr - 0x2800) % 0x0400)];
        }

    } else if (addr <= 0x2FFF) {

        if (nametable_arrangement == VERT) {
            return ppu_memory[0x2400 + ((addr - 0x2C00) % 0x0400)];
        } else {
            return ppu_memory[0x2800 + ((addr - 0x2C00) % 0x0400)];
        }

    } else if (addr <= 0x3EFF) {
        return ppu_read (addr - 0x2000);
    } else if (addr <= 0x3F1F) {
        return ppu_memory[addr];
    } else if (addr <= 0x3FFF) {
        return ppu_memory[0x3F00 + ((addr - 0x3F20) % 0x0020)];
    } else {
        return ppu_read(addr % 0x4000);
    }

}

// Map CPU addresses onto physical addresses for reading purposes
// Emulates default NES address mirroring
void NROM::cpu_write (uint16_t addr, uint8_t data) {

    if (addr <= 0x07FF) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x1FFF) {
        cpu_memory[addr % 0x0800] = data;
    } else if (addr <= 0x2007) {
        write_ppu_from_cpu(addr % 0x0008, data);
    } else if (addr == 0x4014) {
        write_ppu_from_cpu(addr % 16, data);
    } else if (addr <= 0x4017) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x401F) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x5FFF) {
        return;
    } else if (addr <= 0x7FFF) {
        cpu_memory[addr] = data;
    } else if (addr <= 0xFFFF) {
        cpu_memory[addr + ((addr - 0x8000) % prg_rom_size)] = data;
    }

}

// Write to PPU memory, no mapping needed
void NROM::ppu_write (uint16_t addr, uint8_t data) {

    if (addr <= 0x0FFF) {
        ppu_memory[addr] = data;
    } else if (addr <= 0x23FF) {
        ppu_memory[addr] = data;
    } else if (addr <= 0x27FF) {

        if (nametable_arrangement == VERT) {
            ppu_memory[addr] = data;
        } else {
            ppu_memory[0x2000 + ((addr - 0x2400) % 0x0400)] = data;
        }

    } else if (addr <= 0x2BFF) {

        if (nametable_arrangement == HORIZ) {
            ppu_memory[addr] = data;
        } else {
            ppu_memory[0x2000 + ((addr - 0x2800) % 0x0400)] = data;
        }

    } else if (addr <= 0x2FFF) {

        if (nametable_arrangement == VERT) {
            ppu_memory[0x2400 + ((addr - 0x2C00) % 0x0400)] = data;
        } else {
            ppu_memory[0x2800 + ((addr - 0x2C00) % 0x0400)] = data;
        }

    } else if (addr <= 0x3EFF) {
        ppu_write(addr - 0x2000, data);
    } else if (addr <= 0x3F1F) {
        ppu_memory[addr] = data;
    } else if (addr <= 0x3FFF) {
        ppu_memory[0x3F00 + ((addr - 0x3F20) % 0x0020)] = data;;
    } else {
        ppu_write(addr % 0x4000, data);
    }

}


NROM::~NROM () {
}



// Load the rom with given info and filename into a Cartridge object
Cartridge* load_rom (std::string filename) {

	struct ines_info rom_info = parse_rom(filename);

	// TODO: Currently only has support for mapper 0 in iNES format
	switch (rom_info.mapper) {
		case 0:
			return new NROM(rom_info, filename);
		default:
			throw std::runtime_error("Only mapper 0 is currently supported");
	}
}
