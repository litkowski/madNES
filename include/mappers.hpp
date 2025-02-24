#ifndef MAPPER_H
#define MAPPER_H

#include <cstdint>
#include <vector>

class Mapper {
	protected:
		std::vector<uint8_t> memory;
	public:
		virtual uint8_t read (uint16_t address) = 0;
		virtual void write (uint16_t address, uint8_t data) = 0;
};

class NROM_128 : public Mapper {
	public:
		NROM_128 ();
		uint8_t read (uint16_t address);
		void write (uint16_t address, uint8_t data);
};

class NROM_256 : public Mapper {
	public:
		NROM_256 ();
		uint8_t read (uint16_t address);
		void write (uint16_t address, uint8_t data);
};

#endif
