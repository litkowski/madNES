#include <SDL2/SDL.h>
#include "graphics.hpp"
#include <filesystem>
#include <fstream>

SDL_Renderer* renderer;
SDL_Window* window;
uint8_t framebuffer[264][256];

struct color {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

struct color master_palette[0x40];

// Initialize the master palette from a given .pal file
void Init_Master_Palette (std::string filename) {

	// Ensure the file is of type .pal
	std::filesystem::path file_path = filename;

	if (file_path.extension() != ".pal") {
		printf("Palette must be of type .pal.\n");
		throw std::runtime_error("Given palette" + filename + "is not of type .pal.");
	}

	// Read raw palette data into a buffer
	uint8_t palette_raw[192];
	std::ifstream palette_file;
	palette_file.open(filename, std::fstream::in);
	palette_file.read((char*) palette_raw, 192);
	palette_file.close();

	// Convert buffer data into color struct data
	for (int current_palette = 0; current_palette < 0x40; current_palette++) {
		master_palette[current_palette].red = palette_raw[current_palette * 3];
		master_palette[current_palette].green = palette_raw[current_palette * 3 + 1];
		master_palette[current_palette].blue = palette_raw[current_palette * 3 + 2];
	}
}

// Initialize the SDL graphics
void Init_Graphics () {

	// Attempt to initialize needed SDL subsystems, return -1 on failure
    int init = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS);

    if (init != 0) {
        throw std::runtime_error("SDL_Init Error: ");
    }

    // Attempt to create a 1280 x 640 SDL window
    window = SDL_CreateWindow("madNES",
                    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                    1280, 640, SDL_WINDOW_MINIMIZED);

    if (window == NULL) {
        throw std::runtime_error("SDL_WindowCreate Error: ");
		SDL_Quit();
    }

    // Create an SDL Renderer for the 256x240 NES window, abort on failure.
    renderer = SDL_CreateRenderer(window, 0, 0);

    if (renderer == NULL) {
        throw std::runtime_error("SDL_CreateRenderer Error: ");
        SDL_Quit();
	}

    SDL_RenderSetLogicalSize(renderer, 256, 240);
}

// Render the framebuffer to the screen
// Sleep for one 60th of a second after frame
void push_frame_to_screen () {
	for (int cur_x = 0; cur_x < 256; cur_x++) {
		for (int cur_y = 0; cur_y < 240; cur_y++) {
			struct color cur_color = master_palette[framebuffer[cur_x][cur_y]];
			SDL_SetRenderDrawColor(renderer, cur_color.red, cur_color.green, cur_color.green, 255);
			SDL_RenderDrawPoint(renderer, cur_x, cur_y);
		}
	}
	SDL_RenderPresent(renderer);
	// SDL_Delay(16);
}
