#include <stdexcept>

#include "parse_args.hpp"



// Parse a single option argument
error_t nes_parser (int key, char *arg, struct argp_state *state) {

	struct nes_args* nes_args = (struct nes_args*) state->input;
	error_t ret = 0;

	switch (key) {

		// --filename, -fn : Path to the specified
		case 'f': {
			std::string filename = arg;
			nes_args->filename = filename;
			break;
		}

		// --dump, -d : Disassemble and print the ROM's contents to stdout
		case 'd': {
			nes_args->dump_rom = true;
			break;
		}

		default: {
			ret = ARGP_ERR_UNKNOWN;
			break;
		}
	}

	return ret;
}



// Parse arguments to the NES program using GNU argp
struct nes_args parse_nes_args (int argc, char* argv[]) {

	struct nes_args args;

	struct argp_option options[] = {
		{"filename", 'f', "filename", 0, "The path to the desired NES ROM", 0},
		{"dump", 'd', 0, OPTION_ARG_OPTIONAL, "Print a disassembled version of the given ROM to stdout: CURRENTLY UNSUPPORTED", 0},
		{0}
	};

	struct argp argp_settings = {options, nes_parser, 0, 0, 0, 0, 0};

	error_t parsed = argp_parse(&argp_settings, argc, argv, 0, NULL, &args);

	if (parsed != 0) {
		throw std::runtime_error("argp parsing failed");
	}

	return args;
}
