#include "mappers.hpp"

// NOTE: NROM must be extended in the future to handle Family BASIC
NROM_128::NROM_128 () {
	memory = std::vector<uint8_t>(0xFFFF, 0);
}

uint8_t NROM_128::read (uint16_t addr) {
    if (addr <= 0x07FF) {
        return memory[addr];
    } else if (addr <= 0x1FFF) {
        return memory[addr % 0x0800];
    } else if (addr <= 0x2007) {
        return memory[addr];
    } else if (addr <= 0x3FFF) {
        return memory[addr % 0x0008];
    } else if (addr <= 0x4017) {
        return memory[addr];
    } else if (addr <= 0x401F) {
        return memory[addr];
    } else if (addr <= 0xC000) {
        return memory[addr % 0x4000];
    } else if (addr <= 0xFFFF) {
        return memory[addr];
    }
}

void NROM_128::write (uint16_t addr, uint8_t data) {
    if (addr <= 0x07FF) {
        memory[addr] = data;
    } else if (addr <= 0x1FFF) {
        memory[addr % 0x0800] = data;
    } else if (addr <= 0x2007) {
        memory[addr] = data;
    } else if (addr <= 0x3FFF) {
        memory[addr % 0x0008] = data;
    } else if (addr <= 0x4017) {
        memory[addr] = data;
    } else if (addr <= 0x401F) {
        memory[addr] = data;
    } else if (addr <= 0xC000) {
        memory[addr % 0x4000] = data;
    } else if (addr <= 0xFFFF) {
        memory[addr] = data;
    }
}

NROM_256::NROM_256 () {
	memory = std::vector<uint8_t>(0xFFFF, 0);
}

uint8_t NROM_256::read (uint16_t addr) {
    if (addr <= 0x07FF) {
        return memory[addr];
    } else if (addr <= 0x1FFF) {
        return memory[addr % 0x0800];
    } else if (addr <= 0x2007) {
        return memory[addr];
    } else if (addr <= 0x3FFF) {
        return memory[addr % 0x0008];
    } else if (addr <= 0x4017) {
        return memory[addr];
    } else if (addr <= 0x401F) {
        return memory[addr];
    } else if (addr <= 0x7FFF) {
        return memory[addr % 0x8000];
    } else if (addr <= 0xFFFF) {
        return memory[addr];
    }
}

void NROM_256::write (uint16_t addr, uint8_t data) {
    if (addr <= 0x07FF) {
        memory[addr] = data;
    } else if (addr <= 0x1FFF) {
        memory[addr % 0x0800] = data;
    } else if (addr <= 0x2007) {
        memory[addr] = data;
    } else if (addr <= 0x3FFF) {
        memory[addr % 0x0008] = data;
    } else if (addr <= 0x4017) {
        memory[addr] = data;
    } else if (addr <= 0x401F) {
        memory[addr] = data;
    } else if (addr <= 0x7FFF) {
        memory[addr % 0x8000] = data;
    } else if (addr <= 0xFFFF) {
        memory[addr] = data;
    }
}
