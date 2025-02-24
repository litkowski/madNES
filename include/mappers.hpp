#ifndef MAPPER_H
#define MAPPER_H

#include <cstdint>
#include <vector>

class Mapper {
	protected:
		std::vector<uint8_t> cpu_memory;
		std::vector<uint8_t> ppu_memory;
	public:
		virtual uint8_t cpu_read (uint16_t address) = 0;
		virtual uint8_t ppu_read (uint16_t address) = 0;
		virtual void cpu_write (uint16_t address, uint8_t data) = 0;
		virtual void ppu_write (uint16_t address, uint8_t data) = 0;
};

class NROM_128 : public Mapper {
	public:
		NROM_128 ();
		uint8_t cpu_read (uint16_t address);
		uint8_t ppu_read (uint16_t address);
		void cpu_write (uint16_t address, uint8_t data);
		void ppu_write (uint16_t address, uint8_t data);

};

class NROM_256 : public Mapper {
	public:
		NROM_256 ();
		uint8_t cpu_read (uint16_t address);
		uint8_t ppu_read (uint16_t address);
		void cpu_write (uint16_t address, uint8_t data);
		void ppu_write (uint16_t address, uint8_t data);
};

#endif
