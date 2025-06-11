#include "cpu.hpp"
#include "ppu.hpp"
#include "graphics.hpp"
#include <cstring>
#include <iostream>

// PPUCTRL flags
#define NAMETABLE_CTRL 0b00000011
#define VRAM_INCREMENT 0b00000100
#define SPRITE_PATTERN 0b00001000
#define BACKGROUND_PATTERN 0b00010000
#define SPRITE_SIZE 0b00100000
#define MASTER_SLAVE 0b01000000
#define VBLANK_NMI 0b10000000

// PPUMASK flags
#define GREYSCALE 0b00000001
#define SHOW_BACKGROUND_LEFT 0b00000010
#define SHOW_SPRITES_LEFT 0b00000100
#define ENABLE_BACKGROUND 0b00001000
#define ENABLE_SPRITES 0b00010000
#define EMPHASIZE_RED 0b00100000
#define EMPHASIZE_GREEN 0b01000000
#define EMPHASIZE_BLUE 0b10000000

// PPUSTATUS flags
#define PPU_OPEN_BUS 0b00011111
#define SPRITE_OVERFLOW 0b00100000
#define SPRITE_0 0b01000000
#define VBLANK_ACTIVE 0b10000000

// OAM attribute flags
#define SPRITE_PRIORITY 0b00100000
#define FLIP_SPRITE_HORIZ 0b01000000
#define FLIP_SPRITE_VERT 0b10000000

extern uint8_t framebuffer[264][256];
extern Cartridge* game;

struct sprite {
	uint8_t y;
	uint8_t tile_index;
	uint8_t attributes;
	uint8_t x;
};

// Memory mapped registers
uint8_t PPUCTRL;
uint8_t PPUMASK;
uint8_t PPUSTATUS;
uint8_t OAMADDR;
uint8_t OAMDATA;
uint16_t PPUSCROLL;
uint16_t PPUADDR;
uint8_t PPUDATA;
uint8_t OAMDMA;

// Internal registers and memory
uint16_t v;
uint16_t t;
uint8_t w;

// NOTE: May not need to be used, higher level emulation may suffice
// uint8_t x;

int frames;

struct sprite oam[64];

// Current cycle in the frame
uint32_t cycles_left;

// Initialize the PPU
void Init_PPU (Cartridge* mapper) {

	// Reset framebuffer
	std::memset(framebuffer, 0, 264 * 256);

	// Initialize all PPU memory and registers to 0
	std::memset(oam, 0, sizeof(struct sprite) * 64);
	PPUCTRL = PPUMASK = PPUSTATUS = OAMADDR = OAMDATA = PPUDATA = OAMDMA = 0;
	PPUSCROLL = PPUADDR = 0;
	v = t = 0;
	w = 0;
	cycles_left = 0;
	frames = 0;

	// Initialize cartridge
	game = mapper;
}

// Render a single tile on the background
void render_background_tile (uint8_t x, uint8_t y, uint16_t pattern_table_index, uint8_t palette_index) {

	// Copy the universal background color to the local palette
	uint8_t palette_colors[4];
	palette_colors[0] = game->ppu_read(0x3F00);

	// Copy the particular palette colors to the local palette
	for (int i = 1; i < 4; i++) {
		palette_colors[i] = game->ppu_read(0x3F00 + palette_index);
	}

	// Iterate through the tile, setting framebuffer values as needed
	for (int i = 0; i < 8; i++) {

		// Extract the current line's information from the pattern table
		uint8_t tile1 = game->ppu_read(pattern_table_index + i);
		uint8_t tile2 = game->ppu_read(pattern_table_index + i + 8);

		// Render the current line
		for (int j = 0; j < x * 8 + 8; j++) {
			int color_index = ((tile1 & (1 << j)) >> j) + ((tile1 & (1 << j)) >> j) * 2;
			framebuffer[x * 8 + j][y * 8 + i] = palette_colors[color_index];
		}
	}
}


// Render a sprite to the framebuffer
// TODO: Implement sprite 0 hits and sprite overlapping
void render_sprite (struct sprite sprite) {

	// Copy the universal background color to the local palette
	uint8_t palette_colors[4];
	palette_colors[0] = game->ppu_read(0x3F00);

	// Copy the particular palette colors to the local palette
	for (int i = 1; i < 4; i++) {
		palette_colors[i] = game->ppu_read(0x3F00 + sprite.attributes & 0b11 + 4);
	}

	// Extract the currently active sprite size from PPUCTRL
	uint8_t sprite_size = 8 + ((PPUCTRL & SPRITE_SIZE) >> 2);

	// Set the pattern table index
	uint16_t pattern_table_index;
	if (PPUCTRL & SPRITE_SIZE) {
		// Set the index for an 8x16 sprite
		pattern_table_index = (0x1000 * (sprite.tile_index & 0b1)) + ((sprite.tile_index & ~ 0b1) << 4);
	} else {
		// Set the index for an 8x8 sprite
		pattern_table_index = (((PPUCTRL & SPRITE_PATTERN) >> 3) * 0x1000) | (sprite.tile_index << 4);
	}

	// Render the different possible sprites
	switch ((sprite.attributes & 0b11000000) >> 6) {

		// Render a normal sprite
		case 0b00:

			for (int i = 0; i < sprite_size; i++) {

				// Extract the current line's information from the pattern table
				uint8_t tile1 = game->ppu_read(pattern_table_index + i);
				uint8_t tile2 = game->ppu_read(pattern_table_index + i + 8);

				// Render the current line
				for (int j = sprite.x; j < sprite.x + 8; j++) {
					uint8_t color_index = ((tile1 & (1 << j)) >> j) + ((tile2 & (1 << j)) >> j) * 2;
					framebuffer[sprite.x + j][sprite.y + i] = palette_colors[color_index];
				}
			}

			break;

		// Render a horizontally flipped sprite
		case 0b01:

			for (int i = 0; i < sprite_size; i++) {

				// Extract the current line's information from the pattern table
				uint8_t tile1 = game->ppu_read(pattern_table_index + i);
				uint8_t tile2 = game->ppu_read(pattern_table_index + i + 8);

				// Render the current line
				for (int j = sprite.x; j < sprite.x + 8; j++) {
					uint8_t color_index = ((tile1 & (1 << j)) >> j) + ((tile2 & (1 << j)) >> j) * 2;
					framebuffer[sprite.x + 8 - j][sprite.y + i] = palette_colors[color_index];
				}
			}

			break;

		// TODO: Vertical flipping on 8x16 sprites is broken
		// Render a vertically flipped sprite
		case 0b10:

			for (int i = 0; i < sprite_size; i++) {

				// Extract the current line's information from the pattern table
				uint8_t tile1 = game->ppu_read(pattern_table_index + i);
				uint8_t tile2 = game->ppu_read(pattern_table_index + i + 8);

				// Render the current line
				for (int j = sprite.x; j < sprite.x + 8; j++) {
					uint8_t color_index = ((tile1 & (1 << j)) >> j) + ((tile2 & (1 << j)) >> j) * 2;
					framebuffer[sprite.x + j][sprite.y + i + sprite_size - 16] = palette_colors[color_index];
				}
			}

			break;

		// TODO: Vertical flipping on 8x16 sprites is broken
		// Render a double flipped sprite
		case 0b11:

			for (int i = 0; i < sprite_size; i++) {

				// Extract the current line's information from the pattern table
				uint8_t tile1 = game->ppu_read(pattern_table_index + i);
				uint8_t tile2 = game->ppu_read(pattern_table_index + i + 8);

				// Render the current line
				for (int j = sprite.x; j < sprite.x + 8; j++) {
					uint8_t color_index = ((tile1 & (1 << j)) >> j) + ((tile2 & (1 << j)) >> j) * 2;
					framebuffer[sprite.x + 8 - j][sprite.y + i + sprite_size - 16] = palette_colors[color_index];
				}
			}

			break;
	}

}

// Render an entire frame and reset the cycle count
void render_frame () {

	// Reset the framebuffer
	std::memset(framebuffer, 0, 264 * 256);

	// Extract the current scroll nametable
	uint16_t first_tile = 0x2000 + 0x400 * (NAMETABLE_CTRL & PPUCTRL);

	// Find the current scroll position in the nametable from the
	// X and Y scroll register
	first_tile += ((PPUSCROLL & 0xFF00) >> 8) * 32;
	first_tile += PPUSCROLL & 0xFF;

	if (PPUMASK & ENABLE_BACKGROUND) {

		// Loop through all tiles, rendering from OAM and nametables
		for (int i = 0; i < 30; i++) {

			uint16_t cur_line_index = first_tile;
			uint8_t cur_x_index = 0;

			// Color the background
			for (int j = 0; j < 32; j++)

				// Check for wraparound
				if (cur_x_index + (cur_line_index % 32) > 31) {
					if (cur_line_index % 0x800 > 0x400) {
						cur_line_index -= 400;
					} else {
						cur_line_index += 0x400;
					}
					cur_line_index &= 0xF0;
					cur_x_index = 0;
				}

				// Extract the current tile
				uint8_t cur_tile = game->ppu_read(cur_line_index + cur_x_index);

				// Locate the current tile's y-coordinate
				uint8_t cur_y_index = (cur_line_index % 0x400) / 32;

				// Extract the currently chosen palette
				uint8_t cur_palette = game->ppu_read((cur_tile & 0xFC00) + 0x3C0 + (cur_y_index / 4) * 8 + (cur_x_index / 4));
				uint8_t corner = (cur_x_index % 2) / 2 + (cur_y_index % 2);
				uint8_t active_palette = (cur_palette & 0b11 << (corner * 2)) >> (corner * 2);

				// Extract the tile information from the pattern table
				uint16_t pattern_table_index = ((PPUCTRL & BACKGROUND_PATTERN) >> 4) * 0x1000 + (cur_tile << 4);

				// Add the tile to the framebuffer
				render_background_tile(cur_x_index, cur_y_index, pattern_table_index, active_palette);
				cur_x_index++;
			}

			// Render the frame
			first_tile += 32;
			first_tile = first_tile % 0x1000 + 0x2000;
		}

	// Render sprites
	if (PPUMASK & ENABLE_SPRITES) {
		for (int i = 0; i < 64; i++) {
			render_sprite(oam[i]);
		}
	}
}

// Copy a page of memory from the CPU's memory space to PPU OAM
void copy_oamdma (uint8_t address) {

	uint8_t* oam_raw = (uint8_t*) oam;

	// Iterate through all 256 bytes, copying to OAM
	for (int i = 0; i < 256; i++) {
		oam_raw[i] = game->cpu_read(address + i);
	}

	signal_oamdma();
}

// Modify the CPU memory-mapped registers of the PPU
void write_ppu_from_cpu (uint8_t addr, uint8_t data) {
	switch (addr) {
		case 0x0:
			PPUCTRL = data;
			break;
		case 0x1:
			PPUMASK = data;
			break;
		case 0x3:
			OAMADDR = data;
			break;
		case 0x4:
			// NOTE: May be memory unsafe
			OAMDATA = data;
			((uint8_t*) oam)[OAMADDR] = data;
			OAMADDR++;
			break;
		case 0x5:
			PPUSCROLL &= (~(0xF << v));
			PPUSCROLL |= (data << v);
			w ^ 1;
			break;
		case 0x6:
			PPUADDR &= (~(0xF << v));
			PPUADDR |= (data << v);
			w ^ 1;
			break;
		case 0x7:
			PPUDATA = data;
			break;
		case 0x14:
			OAMDMA = data;
			// copy_oamdma(data);
			break;
	}
}

// Modify the CPU memory-mapped registers of the PPU
uint8_t read_ppu_from_cpu (uint8_t addr) {
	switch (addr) {
		case 0x2: {
			uint8_t old = PPUSTATUS;
			PPUSTATUS &= ~VBLANK_ACTIVE;
			return old;
		}
		case 0x4:
			return OAMDATA;
		case 0x7:
			return PPUDATA;
	}
	return 0;
}

// Simulate one cycle of the PPU. If it's time to enter vblank or render
// a new frame, then do so.
void cycle_ppu () {
	if (cycles_left == 0) {
		if (PPUSTATUS & VBLANK_ACTIVE) {
			// signal_nmi();
			PPUSTATUS |= VBLANK_ACTIVE;
			cycles_left = 7140;
		} else {
			frames++;
			std::cout << "Frames rendered: " << frames << "\n";
			render_frame();
			push_frame_to_screen ();
			PPUSTATUS &= ~VBLANK_ACTIVE;
			cycles_left = 81940;
		}
	}
	cycles_left--;
}
