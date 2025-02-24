#include "mappers.hpp"
#include <stdexcept>

// NOTE: NROM must be extended in the future to handle Family BASIC
NROM_128::NROM_128 () {
	cpu_memory = std::vector<uint8_t>(0xFFFF, 0);
    ppu_memory = std::vector<uint8_t>(0x2000, 0);
}

uint8_t NROM_128::cpu_read (uint16_t addr) {
    if (addr <= 0x07FF) {
        return cpu_memory[addr];
    } else if (addr <= 0x1FFF) {
        return cpu_memory[addr % 0x0800];
    } else if (addr <= 0x2007) {
        return cpu_memory[addr];
    } else if (addr <= 0x3FFF) {
        return cpu_memory[addr % 0x0008];
    } else if (addr <= 0x4017) {
        return cpu_memory[addr];
    } else if (addr <= 0x401F) {
        return cpu_memory[addr];
    } else if (addr <= 0xC000) {
        return cpu_memory[addr % 0x4000];
    } else if (addr <= 0xFFFF) {
        return cpu_memory[addr];
    }
}

uint8_t NROM_128::ppu_read (uint16_t addr) {
    if (addr <= 2000) {
        return ppu_memory[addr];
    } else {
        throw std::runtime_error("CHR ROM address" + std::to_string(addr) + "does not exist.");
    }
}

void NROM_128::cpu_write (uint16_t addr, uint8_t data) {
    if (addr <= 0x07FF) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x1FFF) {
        cpu_memory[addr % 0x0800] = data;
    } else if (addr <= 0x2007) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x3FFF) {
        cpu_memory[addr % 0x0008] = data;
    } else if (addr <= 0x4017) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x401F) {
        cpu_memory[addr] = data;
    } else if (addr <= 0xC000) {
        cpu_memory[addr % 0x4000] = data;
    } else if (addr <= 0xFFFF) {
        cpu_memory[addr] = data;
    }
}

void NROM_128::ppu_write (uint16_t addr, uint8_t data) {
    if (addr <= 2000) {
        ppu_memory[addr] = data;
    } else {
        throw std::runtime_error("CHR ROM address" + std::to_string(addr) + "does not exist.");
    }
}

NROM_256::NROM_256 () {
	cpu_memory = std::vector<uint8_t>(0xFFFF, 0);
    ppu_memory = std::vector<uint8_t>(0x2000, 0);
}

uint8_t NROM_256::cpu_read (uint16_t addr) {
    if (addr <= 0x07FF) {
        return cpu_memory[addr];
    } else if (addr <= 0x1FFF) {
        return cpu_memory[addr % 0x0800];
    } else if (addr <= 0x2007) {
        return cpu_memory[addr];
    } else if (addr <= 0x3FFF) {
        return cpu_memory[addr % 0x0008];
    } else if (addr <= 0x4017) {
        return cpu_memory[addr];
    } else if (addr <= 0x401F) {
        return cpu_memory[addr];
    } else if (addr <= 0x7FFF) {
        return cpu_memory[addr % 0x8000];
    } else if (addr <= 0xFFFF) {
        return cpu_memory[addr];
    }
}

uint8_t NROM_256::ppu_read (uint16_t addr) {
    if (addr <= 2000) {
        return ppu_memory[addr];
    } else {
        throw std::runtime_error("CHR ROM address" + std::to_string(addr) + "does not exist.");
    }
}

void NROM_256::cpu_write (uint16_t addr, uint8_t data) {
    if (addr <= 0x07FF) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x1FFF) {
        cpu_memory[addr % 0x0800] = data;
    } else if (addr <= 0x2007) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x3FFF) {
        cpu_memory[addr % 0x0008] = data;
    } else if (addr <= 0x4017) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x401F) {
        cpu_memory[addr] = data;
    } else if (addr <= 0x7FFF) {
        cpu_memory[addr % 0x8000] = data;
    } else if (addr <= 0xFFFF) {
        cpu_memory[addr] = data;
    }
}

void NROM_256::ppu_write (uint16_t addr, uint8_t data) {
    if (addr <= 2000) {
        ppu_memory[addr] = data;
    } else {
        throw std::runtime_error("CHR ROM address" + std::to_string(addr) + "does not exist.");
    }
}
