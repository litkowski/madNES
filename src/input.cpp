#include <SDL2/SDL.h>

uint8_t poll_latch;
uint8_t controller1;
uint8_t current_bit_1;
uint8_t controller2;
uint8_t current_bit_2;

const unsigned char SDL_Keynames[8] = {SDL_SCANCODE_Z, SDL_SCANCODE_X, SDL_SCANCODE_RSHIFT,
	SDL_SCANCODE_RETURN, SDL_SCANCODE_UP, SDL_SCANCODE_DOWN, SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT};

// Signal the I/O system to poll both controllers
// TODO: Implement second controller support
void signal_input_poll () {

	// Poll the keyboard state
	SDL_PumpEvents();
	const unsigned char* SDL_keys = SDL_GetKeyboardState(NULL);

	poll_latch |= 1;

	// Reset the current bits for both controllers
	current_bit_1 = current_bit_2 = 0;

	// Update controller 1's bits in accordance with the keyboard state
	for (int cur_input = 0; cur_input < 8; cur_input++) {
		controller1 |= SDL_keys[SDL_Keynames[cur_input]] << cur_input;
	}
}

// Read from first controller
// TODO: Implement support for non-standard NES controllers
uint8_t read_controller_1 () {

	// Read the current bit of controller 1 into the data line
	uint8_t data_line = 0;
	data_line |= ((controller1 & (1 << current_bit_1)) >> current_bit_1);
	current_bit_1++;
	return data_line;
}

// Read from second controller
// TODO: Implement support for non-standard NES controllers
uint8_t read_controller_2 () {

	// Read the current bit of controller 2 into the data line
	uint8_t data_line = 0;
	data_line |= ((controller2 & (1 << current_bit_2)) >> current_bit_2);
	current_bit_2++;
	return data_line;
}
