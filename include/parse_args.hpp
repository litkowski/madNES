#ifndef PARSE_ARGS_H
#define PARSE_ARGS_H

#include <argp.h>
#include <string>
#include <stdexcept>

struct nes_args {
	std::string filename;
	bool dump_rom = false;
};

struct nes_args parse_nes_args (int argc, char* argv[]);
error_t nes_parser (int key, char* arg, struct argp_state* state);

#endif
