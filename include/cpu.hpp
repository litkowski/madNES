#ifndef CPU_H
#define CPU_H
#include "mappers.hpp"

// Public functions
void Init_CPU (Cartridge* game_cartridge);
void cycle_cpu ();
void signal_nmi ();

#endif
